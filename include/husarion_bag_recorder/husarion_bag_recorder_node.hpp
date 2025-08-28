#ifndef HUSARION_BAG_RECORDER_NODE
#define HUSARION_BAG_RECORDER_NODE
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ctime"

class husarion_bag_recorder_node : public rclcpp::Node {
    struct Flags {
        int saveToBag;
        int reduceOutliers;
        bool allowRestart;
    };

    struct Topics {
        std::string lidarTopic;
        std::string imuLidarTopic;
        std::string joysticTopic;
        std::string vehicleOdometryTopic;
    };

    public:
        husarion_bag_recorder_node();
        void loadParams();
        void setupConnections();
        void startRecording();
        void stopRecording();
        void lidarCallback(std::shared_ptr<rclcpp::SerializedMessage>);
        void vehicleOdomCallback(std::shared_ptr<rclcpp::SerializedMessage>);
        void imuLidarCallback(std::shared_ptr<rclcpp::SerializedMessage>);
        void joystickCallback(const sensor_msgs::msg::Joy &msg);

    private:
        Flags _flags;
        Topics _topics;
        std::string _namespace;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _lidarSubscriber;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _lidarImuSubscriber;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joystickSubscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _vehicleOdomSubscriber;
        rosbag2_cpp::Writer _bagWriter;
};


#endif