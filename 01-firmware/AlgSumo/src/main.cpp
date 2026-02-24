#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class SumoStrategist : public rclcpp::Node {
public:
    SumoStrategist() : Node("sumo_strategist") {
        // Publisher para mover los motores
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscriber para leer sensores (enviados por el ESP32)
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "sensor_data", 10, std::bind(&SumoStrategist::topic_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Nodo de Sumo Iniciado - Esperando sensores...");
    }

private:
    void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        auto drive_cmd = geometry_msgs::msg::Twist();

        // Estructura del mensaje: [Line_Izq, Line_Der, Distancia_Enemigo]
        int line_izq = msg->data[0];
        int line_der = msg->data[1];
        int dist_enemigo = msg->data[2];

        // --- LÓGICA DE ESTADOS ---

        // 1. EMERGENCIA: Detección de línea (Prioridad absoluta)
        if (line_izq < 500 || line_der < 500) {
            drive_cmd.linear.x = -0.6;  // Retroceder rápido
            drive_cmd.angular.z = 1.5;  // Giro brusco
            RCLCPP_WARN(this->get_logger(), "¡LÍNEA DETECTADA! Retrocediendo.");
        }
        // 2. ATAQUE: Enemigo a la vista
        else if (dist_enemigo < 40 && dist_enemigo > 0) {
            drive_cmd.linear.x = 1.0;   // Máxima potencia adelante
            drive_cmd.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "¡OBJETIVO FIJADO! Atacando.");
        }
        // 3. BÚSQUEDA: Escaneo circular
        else {
            drive_cmd.linear.x = 0.0;
            drive_cmd.angular.z = 0.8;  // Rotar para encontrar al rival
        }

        publisher_->publish(drive_cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SumoStrategist>());
    rclcpp::shutdown();
    return 0;
}
