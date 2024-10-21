#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"  // Incluir el servicio estándar SetBool

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher() : Node("simple_publisher"), is_publishing_(true)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SimplePublisher::timer_callback, this));

        // Crear un servicio para activar/desactivar la publicación
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "toggle_publishing",
            std::bind(&SimplePublisher::toggle_publishing, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void timer_callback()
    {
        if (!is_publishing_) {
            return;  // Si no se debe publicar, salir de la función
        }

        auto message = std_msgs::msg::String();
        message.data = "Hola, este es un mensaje desde ROS2 en C++";
        RCLCPP_INFO(this->get_logger(), "Publicando: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    // Función de callback del servicio
    void toggle_publishing(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        is_publishing_ = request->data;
        response->success = true;
        response->message = is_publishing_ ? "Publicación activada" : "Publicación desactivada";
        RCLCPP_INFO(this->get_logger(), "Publicación: %s", response->message.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;  // Servicio para controlar la publicación
    bool is_publishing_;  // Variable para controlar si se debe publicar o no
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}
