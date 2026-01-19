#include "logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <stdarg.h>
#include <stdio.h>

#define LOGGER_QUEUE_LEN 20
#define LOGGER_TASK_STACK 2048

typedef struct {
    log_level_t level;
    char msg[128];
} log_msg_t;

static QueueHandle_t log_queue;

static void logger_task(void* arg)
{
    log_msg_t msg;
    while(1) {
        if(xQueueReceive(log_queue, &msg, portMAX_DELAY)) {
            const char* prefix;
            switch(msg.level) {
                case LOG_INFO: prefix = "INFO"; break;
                case LOG_WARN: prefix = "WARN"; break;
                case LOG_ERROR: prefix = "ERROR"; break;
                default: prefix = "LOG"; break;
            }
            printf("[%s] %s\n", prefix, msg.msg);
        }
    }
}

void logger_init(void)
{
    log_queue = xQueueCreate(LOGGER_QUEUE_LEN, sizeof(log_msg_t));
    xTaskCreate(logger_task, "logger_task", LOGGER_TASK_STACK, NULL, 5, NULL);
}

void logger_send(log_level_t level, const char* fmt, ...)
{
    if(!log_queue) return;

    log_msg_t msg;
    msg.level = level;

    va_list args;
    va_start(args, fmt);
    vsnprintf(msg.msg, sizeof(msg.msg), fmt, args);
    va_end(args);

    xQueueSend(log_queue, &msg, 0);
}
