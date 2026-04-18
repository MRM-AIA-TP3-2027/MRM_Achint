#include "rclcpp/rclcpp.hpp"
#include "cashier_system/srv/get_status.hpp"
#include <iostream>

class StatusViewer : public rclcpp::Node {
public:
    StatusViewer() : Node("status_viewer") {

        client_ = this->create_client<cashier_system::srv::GetStatus>("get_status");

        // 🔥 Input loop
        while (rclcpp::ok()) {
            char c;
            std::cout << "\nPress 's' to get status (or 'q' to quit): ";
            std::cin >> c;

            if (c == 's') {
                send_request();
            }
            else if (c == 'q') {
                RCLCPP_INFO(this->get_logger(), "Exiting Status Viewer...");
                break;
            }
        }
    }

private:
    void send_request() {
        if (!client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }

        auto request = std::make_shared<cashier_system::srv::GetStatus::Request>();

        auto future = client_->async_send_request(request);

        // 🔥 Wait for response (simpler for beginners)
        auto response = future.get();

        RCLCPP_INFO(this->get_logger(), "\n%s", response->status.c_str());
    }

    rclcpp::Client<cashier_system::srv::GetStatus>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StatusViewer>();

    rclcpp::spin_some(node);

    rclcpp::shutdown();
    return 0;
}