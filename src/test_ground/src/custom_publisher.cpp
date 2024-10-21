#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CustomPublisher : public rclcpp::Node
{
public:
    CustomPublisher() : Node("custom_publisher")
    {
        // Declarar el parámetro con un valor por defecto
        this->declare_parameter("publish_frequency", 2.0);

        // Crear el publicador
        publisher_ = this->create_publisher<std_msgs::msg::String>("custom_topic", 10);

        // Configurar el temporizador con la frecuencia basada en el parámetro
        setup_timer();

        // Registrar el callback para cambiar parámetros dinámicamente
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&CustomPublisher::parameter_callback, this, std::placeholders::_1));
    }

private:
    void setup_timer()
    {
        // Obtener el valor del parámetro
        double frequency;
        this->get_parameter("publish_frequency", frequency);

        // Tiempo de publicación (en milisegundos)
        auto period_ms = std::chrono::milliseconds(static_cast<int>(frequency));

        // Crear o recrear el temporizador con la nueva frecuencia
        timer_ = this->create_wall_timer(
            period_ms,
            std::bind(&CustomPublisher::timer_callback, this));
    }

    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Este es un mensaje de prueba con frecuencia modificable";
        RCLCPP_INFO(this->get_logger(), "Publicando: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    // Manejar el cambio de parámetros dinámicamente
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param : parameters)
        {
            if (param.get_name() == "publish_frequency" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                RCLCPP_INFO(this->get_logger(), "Actualizando frecuencia de publicación a: %.2f Hz", param.as_double());
                setup_timer();  // Reconfigura el temporizador con la nueva frecuencia
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomPublisher>());
    rclcpp::shutdown();
    return 0;
}
