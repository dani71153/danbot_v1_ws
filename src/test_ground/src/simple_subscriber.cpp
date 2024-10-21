#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"  // Incluir el servicio estándar SetBool

class SimpleSubscriber : public rclcpp::Node
{
public:
    SimpleSubscriber() : Node("simple_subscriber"), is_subscribed_(false)
    {
        // Crear el servicio para activar/desactivar la suscripción
        toggle_subscription_service_ = this->create_service<std_srvs::srv::SetBool>(
            "toggle_subscription",
            std::bind(&SimpleSubscriber::toggle_subscription, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Función de callback del servicio para activar/desactivar la suscripción
    void toggle_subscription(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data && !is_subscribed_)
        {
            // Crear la suscripción si se solicita activarla
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "topic", 10,
                std::bind(&SimpleSubscriber::topic_callback, this, std::placeholders::_1));
            response->message = "Suscripción activada";
            is_subscribed_ = true;
        }
        else if (!request->data && is_subscribed_)
        {
            // Eliminar la suscripción si se solicita desactivarla
            subscription_.reset();
            response->message = "Suscripción desactivada";
            is_subscribed_ = false;
        }
        else
        {
            response->message = request->data ? "La suscripción ya está activada" : "La suscripción ya está desactivada";
        }

        response->success = true;
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    // Función de callback que se ejecuta al recibir un mensaje
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Recibido: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_subscription_service_;
    bool is_subscribed_;  // Variable para controlar si está suscrito o no
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}
