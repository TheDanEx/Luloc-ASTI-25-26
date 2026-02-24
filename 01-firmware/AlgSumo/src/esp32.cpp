// Cambios necesarios en la configuración del mensaje del ESP32
#include <std_msgs/msg/int32_multi_array.h>

// ... dentro de tu setup de micro-ROS ...
static std_msgs__msg__Int32MultiArray sensor_msg;

// Es vital asignar memoria para el array antes de publicar
sensor_msg.data.capacity = 3; 
sensor_msg.data.size = 3;
sensor_msg.data.data = (int32_t*) malloc(sensor_msg.data.capacity * sizeof(int32_t));

// En tu loop de lectura:
void loop() {
  sensor_msg.data.data[0] = digitalRead(PIN_LINEA_IZQ); // 0 o 1
  sensor_msg.data.data[1] = digitalRead(PIN_LINEA_DER); // 0 o 1
  sensor_msg.data.data[2] = sonar.ping_cm();           // Distancia en cm

  RCSOFTCHECK(rcl_publish(&publisher, &sensor_msg, NULL));
  delay(10); 
}
