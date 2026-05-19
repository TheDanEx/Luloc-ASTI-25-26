#include "mode_interface.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "motor.h"
#include "motor_velocity_ctrl.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "mqtt_custom_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "shared_memory.h"
#include "audio_player.h"
#include "cJSON.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "MODE_SUMO";

#define LIDAR_UART_PORT      UART_NUM_1
#define LIDAR_RX_PIN         GPIO_NUM_14
#define LIDAR_TX_PIN         UART_PIN_NO_CHANGE
#define LIDAR_BAUDRATE       230400
#define LIDAR_HEADER         0x54
#define LIDAR_VER_LEN        0x2C
#define LIDAR_PACKET_SIZE    47
#define LIDAR_POINTS         12
#define WHEEL_BASE_M         0.170f
#define CONFIG_TOPIC         "robot/config/sumo"
#define TIMING_REPORT_CYCLES 250
#define FORWARD_MIN_DEG      290.0f
#define FORWARD_MAX_DEG      110.0f
#define MAX_SCAN_POINTS       512
#define MAX_FORWARD_POINTS    256
#define OBJECT_MAX_DIST_MM    1000

typedef struct {
    float    angle;
    uint16_t distance;
    uint8_t  confidence;
} lidar_point_t;

static lidar_point_t s_buf0[MAX_SCAN_POINTS];
static lidar_point_t s_buf1[MAX_SCAN_POINTS];
static lidar_point_t *s_rx_buf = s_buf0;
static lidar_point_t *s_ctl_buf = s_buf1;
static volatile uint16_t s_rx_count;
static volatile bool     s_scan_ready;
static lidar_point_t s_forward[MAX_FORWARD_POINTS];
static volatile uint16_t s_forward_count;
static portMUX_TYPE s_lidar_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_packets_ok, s_packets_bad, s_bytes_rx;
static volatile float    s_rot_hz;
static volatile uint16_t s_speed_deg_s, s_timestamp_ms;
static volatile int64_t  s_last_rot_us;
static volatile uint32_t s_rotations, s_buf_ovf;
static TaskHandle_t s_lidar_rx_task_handle;
static bool s_uart_initialized;
static volatile bool stop_task;
static bool giro_180;
static uint32_t s_giro_180_start_ms;

typedef struct {
    int64_t min_us, max_us, total_us;
    uint32_t samples, spikes;
} timing_slot_t;

enum { T_EXEC_TOTAL=0, T_SHM_READ, T_SCAN_SWAP, T_FILTER_FWD,
       T_SEARCH, T_KINEMATICS, T_MOTOR_CTRL, T_COUNT };

static timing_slot_t s_t[T_COUNT];
static int64_t t_start_us[T_COUNT];
static volatile uint32_t s_giro_blocked;

static void t0(int s) { t_start_us[s] = esp_timer_get_time(); }
static void t1(int s) {
    int64_t v = esp_timer_get_time() - t_start_us[s];
    if (!s_t[s].samples || v < s_t[s].min_us) s_t[s].min_us = v;
    if (v > s_t[s].max_us) s_t[s].max_us = v;
    if (v > 1000) s_t[s].spikes++;
    s_t[s].total_us += v;
    s_t[s].samples++;
}

typedef struct {
    float kp_w, max_v, max_w, base_v, base_w;
    uint32_t tiempo_giro_180_ms;
    uint8_t umbral_centro;
} sumo_logic_config_t;

static sumo_logic_config_t s_cfg = {
    .kp_w=0.04f,.max_v=0.5f,.max_w=3.0f,.base_v=1.5f,.base_w=0.8f,
    .tiempo_giro_180_ms=2000,.umbral_centro=15
};

static uint16_t r16(const uint8_t *d) { return d[0]|(d[1]<<8); }
static uint32_t ms(void) { return (uint32_t)(esp_timer_get_time()/1000); }
static bool fwd_ang(float a) {
    if (a < 0) { a += 360.0f; }
    if (a >= 360.0f) { a -= 360.0f; }
    return a >= FORWARD_MIN_DEG || a <= FORWARD_MAX_DEG;
}

static void lidar_rx_task(void *arg) {
    (void)arg; uint8_t p[LIDAR_PACKET_SIZE]; int pos=0;
    uint16_t last_end=0xFFFF; bool first=true;
    while(!stop_task){
        uint8_t b;
        if(uart_read_bytes(LIDAR_UART_PORT,&b,1,pdMS_TO_TICKS(20))<=0){vTaskDelay(1);continue;}
        portENTER_CRITICAL(&s_lidar_mux); s_bytes_rx++; portEXIT_CRITICAL(&s_lidar_mux);
        if(pos==0){if(b==LIDAR_HEADER){p[0]=b;pos=1;}continue;}
        if(pos==1){if(b==LIDAR_VER_LEN){p[1]=b;pos=2;}else if(b==LIDAR_HEADER){p[0]=b;pos=1;}else pos=0;continue;}
        p[pos++]=b; if(pos<LIDAR_PACKET_SIZE)continue; pos=0;
        if(p[0]!=LIDAR_HEADER||p[1]!=LIDAR_VER_LEN){
            portENTER_CRITICAL(&s_lidar_mux);s_packets_bad++;portEXIT_CRITICAL(&s_lidar_mux);continue;
        }
        uint16_t sa=r16(&p[4]),ea=r16(&p[42]);
        if(sa>=36000||ea>=36000){
            portENTER_CRITICAL(&s_lidar_mux);s_packets_bad++;portEXIT_CRITICAL(&s_lidar_mux);continue;
        }
        portENTER_CRITICAL(&s_lidar_mux);
        s_packets_ok++; s_speed_deg_s=r16(&p[2]); s_timestamp_ms=r16(&p[44]);
        if(!first&&ea<last_end){
            s_scan_ready=true; s_rotations++;
            int64_t now=esp_timer_get_time();
            if(s_last_rot_us>0){
                float dt=(now-s_last_rot_us)/1000000.0f;
                if(dt>0)s_rot_hz=1.0f/dt;
            }
            s_last_rot_us=now;
        }
        first=false; last_end=ea;
        float sd=sa/100.0f,ed=ea/100.0f,ei=ed; if(ei<sd)ei+=360.0f;
        float step=(ei-sd)/(float)(LIDAR_POINTS-1);
        for(int i=0;i<LIDAR_POINTS;i++){
            int o=6+i*3; float ang=sd+step*i; if(ang>=360.0f)ang-=360.0f;
            if(s_rx_count<MAX_SCAN_POINTS){
                s_rx_buf[s_rx_count].angle=ang;
                s_rx_buf[s_rx_count].distance=r16(&p[o]);
                s_rx_buf[s_rx_count].confidence=p[o+2];
                s_rx_count++;
            }else s_buf_ovf++;
        }
        portEXIT_CRITICAL(&s_lidar_mux);
    }
    s_lidar_rx_task_handle=NULL; vTaskDelete(NULL);
}

static esp_err_t lidar_uart_start(void) {
    if(s_uart_initialized){uart_flush_input(LIDAR_UART_PORT);return ESP_OK;}
    uart_config_t c={.baud_rate=LIDAR_BAUDRATE,.data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    esp_err_t e=uart_driver_install(LIDAR_UART_PORT,8192,0,0,NULL,0);
    if(e!=ESP_OK&&e!=ESP_ERR_INVALID_STATE){ESP_LOGE(TAG,"uart fail");return e;}
    e=uart_param_config(LIDAR_UART_PORT,&c); if(e!=ESP_OK)return e;
    e=uart_set_pin(LIDAR_UART_PORT,LIDAR_TX_PIN,LIDAR_RX_PIN,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    if(e!=ESP_OK)return e;
    uart_flush_input(LIDAR_UART_PORT); s_uart_initialized=true;
    ESP_LOGI(TAG,"LiDAR GPIO%d @%d",LIDAR_RX_PIN,LIDAR_BAUDRATE);
    return ESP_OK;
}
static void lidar_tasks_start(void) {
    stop_task=false;
    if(!s_lidar_rx_task_handle) xTaskCreatePinnedToCore(lidar_rx_task,"lidar_rx",4096,NULL,2,&s_lidar_rx_task_handle,1);
}
static void lidar_tasks_stop(void) {
    stop_task=true; uart_flush_input(LIDAR_UART_PORT);
    for(int i=0;i<20;i++){if(!s_lidar_rx_task_handle)break;vTaskDelay(pdMS_TO_TICKS(10));}
}

static bool find_enemy(lidar_point_t *pts,uint16_t n,uint16_t *pos,uint16_t *dist) {
    if(!n)return false;
    uint16_t best=UINT16_MAX; bool f=false; *dist=0; *pos=0;
    for(uint16_t i=0;i<n;i++){
        if(!pts[i].distance||pts[i].distance>=best)continue;
        uint16_t pi=i; while(i<n&&pts[i].distance<OBJECT_MAX_DIST_MM)i++;
        uint16_t pe=i; uint32_t s=0;uint16_t c=0;
        for(uint16_t j=pi;j<pe;j++) if(pts[j].distance){s+=pts[j].distance;c++;}
        if(!c)continue;
        uint16_t avg=s/c;
        if(avg<best){*pos=(pe+pi)/2;*dist=avg;best=avg;f=true;}
    }
    return f;
}

static void sumo(float *vL,float *vR) {
    uint16_t po=0,d0=0;
    t0(T_SEARCH);
    bool found=(s_forward_count>0)?find_enemy(s_forward,s_forward_count,&po,&d0):false;
    t1(T_SEARCH);
    t0(T_KINEMATICS);
    float v=0,w=0;
    if(!found){ w=s_cfg.base_w; if(!w)w=0.5f; }
    else {
        int c=s_forward_count/2,d=c-(int)po;
        if(d>-(int)s_cfg.umbral_centro&&d<(int)s_cfg.umbral_centro){w=0;v=s_cfg.max_v;}
        else{
            float bw=d<0?-s_cfg.base_w:s_cfg.base_w;
            w=bw+s_cfg.kp_w*d;
            if(w>s_cfg.max_w){w=s_cfg.max_w;}
            if(w<-s_cfg.max_w){w=-s_cfg.max_w;}
        }
    }
    *vL=v-(w*WHEEL_BASE_M/2.0f); *vR=v+(w*WHEEL_BASE_M/2.0f);
    t1(T_KINEMATICS);
}

static uint32_t s_exec_cycle;

static void timing_report(void) {
    const char *nm[]={"exec_total","shm_read","scan_swap","filter_fwd",
                      "search","kinematics","motor_ctrl"};
    printf("\n========== SUMO TIMING (%lu cyc) ==========\n",
           (unsigned long)s_t[0].samples);
    printf("%-16s %8s %8s %8s %6s %6s\n","SECTION","MIN(us)","AVG(us)","MAX(us)","SMP","SPIKE");
    printf("-------------------------------------------------------------------\n");
    for(int i=0;i<T_COUNT;i++){
        if(!s_t[i].samples)continue;
        printf("%-16s %8lld %8lld %8lld %6lu %6lu\n",nm[i],
            (long long)s_t[i].min_us,
            (long long)(s_t[i].total_us/s_t[i].samples),
            (long long)s_t[i].max_us,
            (unsigned long)s_t[i].samples,
            (unsigned long)s_t[i].spikes);
    }
    printf("-------------------------------------------------------------------\n");
    uint32_t pok,pbad,rx,rots,ovf; float hz; uint16_t speed,ts,fwd;
    portENTER_CRITICAL(&s_lidar_mux);
    pok=s_packets_ok; pbad=s_packets_bad; rx=s_bytes_rx;
    rots=s_rotations; hz=s_rot_hz; speed=s_speed_deg_s;
    ts=s_timestamp_ms; fwd=s_forward_count; ovf=s_buf_ovf;
    portEXIT_CRITICAL(&s_lidar_mux);
    printf("LiDAR: fwd=%u ok=%lu bad=%lu ovf=%lu rx=%luB rpm=%u hz=%.1f rots=%lu\n",
           fwd,(unsigned long)pok,(unsigned long)pbad,(unsigned long)ovf,(unsigned long)rx,
           speed/6, hz,(unsigned long)rots);
    printf("Giro180: blocked=%lu\n",(unsigned long)s_giro_blocked);
    printf("==================================================\n\n");
    memset(s_t,0,sizeof(s_t)); s_giro_blocked=0;
}

static void mqtt_config_cb(const char *t,int tl,const char *d,int dl) {
    if(!d||dl<=0||dl>1024)return;
    cJSON *r=cJSON_ParseWithLength(d,dl); if(!r)return;
    cJSON *kpw=cJSON_GetObjectItem(r,"kp_w"),*mxv=cJSON_GetObjectItem(r,"max_v");
    cJSON *mxw=cJSON_GetObjectItem(r,"max_w"),*bsv=cJSON_GetObjectItem(r,"base_v");
    cJSON *bsw=cJSON_GetObjectItem(r,"base_w"),*tg=cJSON_GetObjectItem(r,"tiempo_giro_180_ms");
    cJSON *uc=cJSON_GetObjectItem(r,"umbral_centro");
    if(kpw){s_cfg.kp_w=kpw->valuedouble;}
    if(mxv){s_cfg.max_v=mxv->valuedouble;}
    if(mxw){s_cfg.max_w=mxw->valuedouble;}
    if(bsv){s_cfg.base_v=bsv->valuedouble;}
    if(bsw){s_cfg.base_w=bsw->valuedouble;}
    if(cJSON_IsNumber(tg)&&tg->valueint>=0){s_cfg.tiempo_giro_180_ms=(uint32_t)tg->valueint;}
    if(uc){s_cfg.umbral_centro=(uint8_t)uc->valueint;}
    cJSON_Delete(r);
}

static void enter(void) {
    ESP_LOGI(TAG,"Enter SUMO (ptr-swap, 220deg)");
    portENTER_CRITICAL(&s_lidar_mux);
    memset(s_buf0,0,sizeof(s_buf0)); memset(s_buf1,0,sizeof(s_buf1));
    s_rx_buf=s_buf0; s_ctl_buf=s_buf1;
    s_rx_count=0; s_scan_ready=false; s_forward_count=0;
    s_packets_ok=s_packets_bad=s_bytes_rx=0;
    s_rotations=0; s_rot_hz=0; s_buf_ovf=0; s_last_rot_us=0;
    portEXIT_CRITICAL(&s_lidar_mux);
    mqtt_custom_client_register_topic_callback(CONFIG_TOPIC,mqtt_config_cb);
    if(mqtt_custom_client_is_connected()) mqtt_custom_client_subscribe(CONFIG_TOPIC,0);
    if(lidar_uart_start()==ESP_OK) lidar_tasks_start();
    memset(s_t,0,sizeof(s_t)); s_exec_cycle=0; s_giro_blocked=0; giro_180=false;
}

static void execute(motor_driver_mcpwm_t *motors,
                    motor_velocity_ctrl_handle_t cl,
                    motor_velocity_ctrl_handle_t cr, float dt) {
    t0(T_EXEC_TOTAL); (void)dt; float vL,vR;
    t0(T_SHM_READ);
    shared_memory_t *shm=shared_memory_get();
    if(!shm){t1(T_SHM_READ);t1(T_EXEC_TOTAL);return;}
    if(xSemaphoreTake(shm->mutex,pdMS_TO_TICKS(10))!=pdTRUE){t1(T_SHM_READ);t1(T_EXEC_TOTAL);return;}
    bool det=shm->sensors.line_detected;
    float curl=shm->sensors.motor_speed_left, curr=-shm->sensors.motor_speed_right;
    float bat=shm->sensors.battery_voltage;
    xSemaphoreGive(shm->mutex);
    t1(T_SHM_READ);

    /* Pointer swap on rotation (NO memcpy) */
    t0(T_SCAN_SWAP);
    bool has_scan=false; uint16_t n_pts=0;
    if(s_scan_ready){
        portENTER_CRITICAL(&s_lidar_mux);
        s_scan_ready=false;
        lidar_point_t *tmp=s_ctl_buf;
        s_ctl_buf=s_rx_buf;
        s_rx_buf=tmp;
        n_pts=s_rx_count;
        s_rx_count=0;
        has_scan=true;
        portEXIT_CRITICAL(&s_lidar_mux);
    }
    t1(T_SCAN_SWAP);

    /* Filter 220deg from ctl_buf (NO memcpy needed) */
    t0(T_FILTER_FWD);
    if(has_scan){
        uint16_t n=0;
        for(uint16_t i=0;i<n_pts&&n<MAX_FORWARD_POINTS;i++){
            if(fwd_ang(s_ctl_buf[i].angle)){
                s_forward[n++]=s_ctl_buf[i];
            }
        }
        s_forward_count=n;
    }
    t1(T_FILTER_FWD);

    if(det&&!giro_180){giro_180=true;s_giro_180_start_ms=ms();}
    if(giro_180){
        uint32_t el=ms()-s_giro_180_start_ms;
        if(el>=s_cfg.tiempo_giro_180_ms){giro_180=false;vL=0;vR=0;}
        else if(el<400){vL=-s_cfg.base_v;vR=-s_cfg.base_v;}
        else{
            float w=s_cfg.max_w;
            if(!w)w=s_cfg.base_w>0?s_cfg.base_w:0.8f;
            vL=-w*WHEEL_BASE_M/2.0f;vR=w*WHEEL_BASE_M/2.0f;
        }
        s_giro_blocked++;
    }else{
        sumo(&vL,&vR);
    }

    t0(T_MOTOR_CTRL);
    motor_velocity_input_t ml={.target_speed=vL,.current_speed=curl,.battery_mv=bat};
    motor_velocity_input_t mr={.target_speed=vR,.current_speed=curr,.battery_mv=bat};
    float pl,pr;
    motor_velocity_ctrl_update(cl,&ml,dt,&pl,NULL);
    motor_velocity_ctrl_update(cr,&mr,dt,&pr,NULL);
    motor_mcpwm_set(motors,(int16_t)(pl*10.0f),(int16_t)(pr*10.0f));
    t1(T_MOTOR_CTRL);
    t1(T_EXEC_TOTAL);
    if(++s_exec_cycle>=TIMING_REPORT_CYCLES){s_exec_cycle=0;timing_report();}
}

static void exit_mode(motor_driver_mcpwm_t *m){
    ESP_LOGI(TAG,"Exit SUMO");
    motor_mcpwm_stop(m); lidar_tasks_stop(); giro_180=false;
}

const mode_interface_t mode_sumo={.enter=enter,.execute=execute,.exit=exit_mode};
