#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"
#include <iostream>

class BillGenerator : public rclcpp::Node {
public:
    BillGenerator() : Node("bill_generator") {
        publisher_ = this->create_publisher<cashier_system::msg::Bill>("bill", 10);

        while (rclcpp::ok()) {
            if (!publish_bill()) {
                break;
            }
        }
    }

private:
    bool publish_bill() {
        auto msg = cashier_system::msg::Bill();  // message object

        std::string item;
        int quantity;
        float price;

        std::cout << "\nEnter item name (or type 'exit'): ";
        std::cin >> item;

        if (item == "exit") {
            RCLCPP_INFO(this->get_logger(), "Exiting Bill Generator...");
            return false;
        }

        std::cout << "Enter quantity: ";
        std::cin >> quantity;

        std::cout << "Enter price: ";
        std::cin >> price;

        msg.item_name = item;
        msg.quantity = quantity;
        msg.price = price;

        RCLCPP_INFO(this->get_logger(),
                    "Publishing bill: %s x%d @ %.2f",
                    msg.item_name.c_str(), msg.quantity, msg.price);

        publisher_->publish(msg);

        return true;
    }

    rclcpp::Publisher<cashier_system::msg::Bill>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BillGenerator>();  // create node object 
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}