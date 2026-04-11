#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"
#include "cashier_system/srv/get_status.hpp"

#include <unordered_map>

class InventoryManager : public rclcpp::Node {
public:
    InventoryManager() : Node("inventory_manager"), total_income_(0.0) {

        // 🔵 Subscriber: receive bills
        subscription_ = this->create_subscription<cashier_system::msg::Bill>(
            "bill", 10,
            std::bind(&InventoryManager::bill_callback, this, std::placeholders::_1));

        // 🟡 Service: provide status when requested
        service_ = this->create_service<cashier_system::srv::GetStatus>(
            "get_status",
            std::bind(&InventoryManager::handle_status, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Inventory Manager Started");
    }

private:
    // 🔥 Runs whenever a bill is received
    void bill_callback(const cashier_system::msg::Bill::SharedPtr msg) {
        std::string item = msg->item_name;
        int quantity = msg->quantity;
        float price = msg->price;

        // Update inventory
        inventory_[item] += quantity;

        // Update total income
        total_income_ += quantity * price;

        RCLCPP_INFO(this->get_logger(),
            "Received: %s x%d @ %.2f", item.c_str(), quantity, price);
    }

    // 🔥 Runs when service is called (press 's')
    void handle_status(
        const std::shared_ptr<cashier_system::srv::GetStatus::Request>,
        std::shared_ptr<cashier_system::srv::GetStatus::Response> response)
    {
        std::string result = "Inventory:\n";

        // Loop through inventory
        for (auto &i : inventory_) {
            result += i.first + ": " + std::to_string(i.second) + "\n";
        }

        // Add total income
        result += "Total Income: " + std::to_string(total_income_);

        // Send response
        response->status = result;
    }

    // 🔵 Subscriber
    rclcpp::Subscription<cashier_system::msg::Bill>::SharedPtr subscription_;

    // 🟡 Service
    rclcpp::Service<cashier_system::srv::GetStatus>::SharedPtr service_;

    // 🧠 Stored data (shared state)
    std::unordered_map<std::string, int> inventory_;
    float total_income_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InventoryManager>());
    rclcpp::shutdown();
    return 0;
}