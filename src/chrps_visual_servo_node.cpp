#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <memory>
#include <thread>
#include <cstdlib>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

class CHRPSVisualServoNode : public rclcpp::Node
{
public:
    CHRPSVisualServoNode() : Node("chrps_visual_servo_node")
    {
        // Declare parameters
        this->declare_parameter("robot_ip", "172.16.0.2");
        this->declare_parameter("tag_size", 0.05);
        this->declare_parameter("desired_factor", 5.0);
        this->declare_parameter("verbose", false);

        // Get parameters
        robot_ip_ = this->get_parameter("robot_ip").as_string();
        tag_size_ = this->get_parameter("tag_size").as_double();
        desired_factor_ = this->get_parameter("desired_factor").as_double();
        verbose_ = this->get_parameter("verbose").as_bool();

        // Publishers
        status_pub_ = this->create_publisher<std_msgs::msg::Bool>("servo_active", 10);

        // Subscribers
        activation_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "activate_servo", 10,
            std::bind(&CHRPSVisualServoNode::activation_callback, this, std::placeholders::_1));

        // Service
        servo_service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_servo_active",
            std::bind(&CHRPSVisualServoNode::servo_service_callback, this, 
                     std::placeholders::_1, std::placeholders::_2));

        // Initialize state
        servo_active_ = false;
        servo_pid_ = -1;

        RCLCPP_INFO(this->get_logger(), "CHRPS Visual Servo Node initialized");
        RCLCPP_INFO(this->get_logger(), "Robot IP: %s", robot_ip_.c_str());
        RCLCPP_INFO(this->get_logger(), "Tag size: %.3f m", tag_size_);
        RCLCPP_INFO(this->get_logger(), "Desired factor: %.1f", desired_factor_);
        RCLCPP_INFO(this->get_logger(), "Ready to launch standalone visual servo on activation");
    }

    ~CHRPSVisualServoNode()
    {
        stop_servo();
    }

private:
    void activation_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !servo_active_) {
            start_servo();
        } else if (!msg->data && servo_active_) {
            stop_servo();
        }
    }

    void servo_service_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data && !servo_active_) {
            start_servo();
            response->success = servo_active_;
            response->message = servo_active_ ? "Visual servo started" : "Failed to start visual servo";
        } else if (!request->data && servo_active_) {
            stop_servo();
            response->success = !servo_active_;
            response->message = !servo_active_ ? "Visual servo stopped" : "Failed to stop visual servo";
        } else {
            response->success = true;
            response->message = servo_active_ ? "Visual servo already active" : "Visual servo already inactive";
        }
    }

    void start_servo()
    {
        if (servo_active_) {
            RCLCPP_WARN(this->get_logger(), "Visual servo already active");
            return;
        }

        // Build command to launch standalone executable
        std::string executable_path = "./build/servoFrankaIBVS_CHRPS";
        std::string command = executable_path;
        
        // Add parameters
        command += " --ip " + robot_ip_;
        command += " --tag-size " + std::to_string(tag_size_);
        command += " --desired-factor " + std::to_string(desired_factor_);
        if (verbose_) {
            command += " --verbose";
        }

        RCLCPP_INFO(this->get_logger(), "Starting standalone visual servo: %s", command.c_str());

        // Fork and exec the standalone process
        servo_pid_ = fork();
        if (servo_pid_ == 0) {
            // Child process - execute standalone
            execl("/bin/bash", "bash", "-c", command.c_str(), (char*)NULL);
            // If we get here, exec failed
            RCLCPP_ERROR(rclcpp::get_logger("chrps_visual_servo"), "Failed to exec standalone process");
            exit(1);
        } else if (servo_pid_ > 0) {
            // Parent process - servo started successfully
            servo_active_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Visual servo started (PID: %d)", servo_pid_);
            
            // Publish status
            auto status_msg = std_msgs::msg::Bool();
            status_msg.data = true;
            status_pub_->publish(status_msg);
            
            // Start monitoring thread
            monitor_thread_ = std::thread(&CHRPSVisualServoNode::monitor_servo_process, this);
        } else {
            // Fork failed
            RCLCPP_ERROR(this->get_logger(), "Failed to fork process for visual servo");
        }
    }

    void stop_servo()
    {
        if (!servo_active_ || servo_pid_ <= 0) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Stopping visual servo (PID: %d)", servo_pid_);

        // Send SIGTERM to gracefully stop the process
        if (kill(servo_pid_, SIGTERM) == 0) {
            // Wait up to 5 seconds for graceful shutdown
            for (int i = 0; i < 50; i++) {
                int status;
                pid_t result = waitpid(servo_pid_, &status, WNOHANG);
                if (result == servo_pid_) {
                    RCLCPP_INFO(this->get_logger(), "Visual servo stopped gracefully");
                    break;
                } else if (result == -1) {
                    RCLCPP_WARN(this->get_logger(), "Error waiting for process");
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // If still running, force kill
            if (kill(servo_pid_, 0) == 0) {
                RCLCPP_WARN(this->get_logger(), "Force killing visual servo process");
                kill(servo_pid_, SIGKILL);
                waitpid(servo_pid_, nullptr, 0);
            }
        }

        servo_active_ = false;
        servo_pid_ = -1;
        
        // Wait for monitor thread to finish
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }

        RCLCPP_INFO(this->get_logger(), "Visual servo stopped");
        
        // Publish status
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = false;
        status_pub_->publish(status_msg);
    }

    void monitor_servo_process()
    {
        if (servo_pid_ <= 0) return;

        int status;
        pid_t result = waitpid(servo_pid_, &status, 0);
        
        if (result == servo_pid_) {
            if (WIFEXITED(status)) {
                int exit_code = WEXITSTATUS(status);
                if (exit_code == 0) {
                    RCLCPP_INFO(this->get_logger(), "Visual servo completed successfully");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Visual servo exited with code: %d", exit_code);
                }
            } else if (WIFSIGNALED(status)) {
                int signal = WTERMSIG(status);
                RCLCPP_WARN(this->get_logger(), "Visual servo terminated by signal: %d", signal);
            }
        }

        // Update state
        servo_active_ = false;
        servo_pid_ = -1;
        
        // Publish status
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = false;
        status_pub_->publish(status_msg);
    }

    // ROS 2 interface
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activation_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr servo_service_;

    // Parameters
    std::string robot_ip_;
    double tag_size_;
    double desired_factor_;
    bool verbose_;

    // State variables
    bool servo_active_;
    pid_t servo_pid_;
    std::thread monitor_thread_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CHRPSVisualServoNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}