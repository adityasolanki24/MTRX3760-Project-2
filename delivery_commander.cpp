// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: delivery_commander.cpp
// Author(s): Project Team
//
// Interactive delivery command interface for ArUco tag-based navigation.
// Allows user to input ArUco tag IDs and commands robot to navigate to them.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

class DeliveryCommander : public rclcpp::Node {
private:
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_tag_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr delivery_status_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;
    
    // Delivery tracking
    int current_target_tag_;
    std::string current_status_;
    int completed_deliveries_;
    
    // Delivery log
    std::string delivery_log_file_;
    
    // User input thread
    std::thread input_thread_;
    bool running_;
    
public:
    DeliveryCommander() 
        : Node("delivery_commander"),
          current_target_tag_(-1),
          current_status_("IDLE"),
          completed_deliveries_(0),
          delivery_log_file_("/tmp/aruco_delivery_log.txt"),
          running_(true)
    {
        // Publisher for target ArUco tag ID
        target_tag_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "target_aruco_tag", 10);
        
        // Publisher for delivery status updates
        delivery_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "delivery_status", 10);
        
        // Subscriber for navigation status
        nav_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "navigation_status", 10,
            std::bind(&DeliveryCommander::navigationStatusCallback, this, 
                     std::placeholders::_1));
        
        // Initialize delivery log
        initializeDeliveryLog();
        
        RCLCPP_INFO(this->get_logger(), "Delivery Commander Started");
        RCLCPP_INFO(this->get_logger(), "Delivery log: %s", delivery_log_file_.c_str());
        
        std::cout << "\nDelivery Commander Ready\n";
        std::cout << "Enter ArUco marker ID to navigate (or 'help' for commands)\n\n";
        
        // Start input thread
        input_thread_ = std::thread(&DeliveryCommander::inputLoop, this);
    }
    
    ~DeliveryCommander() {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }
    
private:
    void initializeDeliveryLog() {
        std::ofstream log_file(delivery_log_file_, std::ios::out);
        if (log_file.is_open()) {
            log_file << "==========================================\n";
            log_file << "ARUCO DELIVERY COMMANDER - LOG\n";
            log_file << "==========================================\n";
            log_file << "System: ArUco Tag-Based Navigation\n";
            log_file << "Log Started: " << getCurrentTimestamp() << "\n";
            log_file << "==========================================\n\n";
            log_file.close();
            RCLCPP_INFO(this->get_logger(), "Delivery log initialized successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to create delivery log file!");
        }
    }
    
    void printHelp() {
        std::cout << "\nAvailable Commands:\n";
        std::cout << "  <number>  - Navigate to ArUco tag ID (e.g., 0, 1, 3)\n";
        std::cout << "  status    - Show current status\n";
        std::cout << "  log       - View delivery log\n";
        std::cout << "  help      - Show this help\n";
        std::cout << "  quit      - Exit program\n\n";
    }
    
    void inputLoop() {
        std::string input;
        
        while (running_ && rclcpp::ok()) {
            std::cout << "Command> ";
            std::cout.flush();
            
            if (!std::getline(std::cin, input)) {
                break;
            }
            
            // Trim whitespace
            input.erase(0, input.find_first_not_of(" \t\n\r"));
            input.erase(input.find_last_not_of(" \t\n\r") + 1);
            
            if (input.empty()) {
                continue;
            }
            
            processCommand(input);
        }
    }
    
    void processCommand(const std::string& command) {
        if (command == "quit" || command == "exit" || command == "q") {
            RCLCPP_INFO(this->get_logger(), "Shutting down delivery commander...");
            running_ = false;
            rclcpp::shutdown();
            return;
        }
        
        if (command == "help" || command == "h" || command == "?") {
            printHelp();
            return;
        }
        
        if (command == "status" || command == "s") {
            printStatus();
            return;
        }
        
        if (command == "log" || command == "l") {
            displayLog();
            return;
        }
        
        // Try to parse as ArUco tag ID
        try {
            int tag_id = std::stoi(command);
            
            if (tag_id < 0) {
                std::cout << "Error: Tag ID must be non-negative\n";
                return;
            }
            
            sendDeliveryCommand(tag_id);
            
        } catch (const std::exception& e) {
            std::cout << "Unknown command: '" << command << "'. Type 'help' for commands.\n";
        }
    }
    
    void sendDeliveryCommand(int tag_id) {
        current_target_tag_ = tag_id;
        current_status_ = "NAVIGATING";
        
        // Publish target tag ID
        auto msg = std_msgs::msg::Int32();
        msg.data = tag_id;
        target_tag_pub_->publish(msg);
        
        // Publish status update
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "STARTED:" + std::to_string(tag_id);
        delivery_status_pub_->publish(status_msg);
        
        std::cout << "Navigating to ArUco tag " << tag_id << "\n";
        
        logDeliveryEvent(tag_id, "STARTED", "Command sent to navigation system");
        
        RCLCPP_INFO(this->get_logger(), 
            "Delivery command sent: Navigate to ArUco tag %d", tag_id);
    }
    
    void navigationStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string status = msg->data;
        
        RCLCPP_INFO(this->get_logger(), "Navigation status: %s", status.c_str());
        
        // Check if arrived at target
        if (status.find("ARRIVED") != std::string::npos || 
            status.find("REACHED") != std::string::npos) {
            
            if (current_target_tag_ >= 0) {
                completed_deliveries_++;
                
                std::cout << "Arrived at ArUco tag " << current_target_tag_ 
                         << " (Total deliveries: " << completed_deliveries_ << ")\n";
                
                logDeliveryEvent(current_target_tag_, "COMPLETED", 
                               "Successfully reached target");
                
                current_status_ = "IDLE";
                current_target_tag_ = -1;
            }
        }
        else if (status.find("FAILED") != std::string::npos || 
                 status.find("ERROR") != std::string::npos) {
            
            std::cout << "Navigation failed: " << status << "\n";
            
            if (current_target_tag_ >= 0) {
                logDeliveryEvent(current_target_tag_, "FAILED", status);
            }
            
            current_status_ = "IDLE";
            current_target_tag_ = -1;
        }
        else if (status.find("SEARCHING") != std::string::npos) {
            current_status_ = "SEARCHING";
            std::cout << "Searching for ArUco tag " << current_target_tag_ << "\n";
        }
    }
    
    void printStatus() {
        std::cout << "\nStatus: " << current_status_ << "\n";
        
        if (current_target_tag_ >= 0) {
            std::cout << "Target Tag: " << current_target_tag_ << "\n";
        } else {
            std::cout << "Target Tag: None\n";
        }
        
        std::cout << "Completed Deliveries: " << completed_deliveries_ << "\n\n";
    }
    
    void displayLog() {
        std::cout << "\nDelivery Log:\n";
        std::cout << "----------------------------------------\n";
        
        std::ifstream log_file(delivery_log_file_);
        if (log_file.is_open()) {
            std::string line;
            while (std::getline(log_file, line)) {
                std::cout << line << "\n";
            }
            log_file.close();
        } else {
            std::cout << "Could not open log file\n";
        }
        
        std::cout << "----------------------------------------\n\n";
    }
    
    void logDeliveryEvent(int tag_id, const std::string& event, 
                         const std::string& details) {
        std::ofstream log_file(delivery_log_file_, std::ios::app);
        if (log_file.is_open()) {
            log_file << "Delivery to ArUco Tag: " << tag_id << "\n";
            log_file << "Event: " << event << "\n";
            log_file << "Details: " << details << "\n";
            log_file << "Timestamp: " << getCurrentTimestamp() << "\n";
            log_file << "Total Completed: " << completed_deliveries_ << "\n";
            log_file << "------------------------------------------\n\n";
            log_file.close();
        }
    }
    
    std::string getCurrentTimestamp() const {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto commander = std::make_shared<DeliveryCommander>();
    
    // Spin in separate thread to allow input processing
    std::thread spin_thread([commander]() {
        rclcpp::spin(commander);
    });
    
    spin_thread.join();
    
    rclcpp::shutdown();
    return 0;
}

