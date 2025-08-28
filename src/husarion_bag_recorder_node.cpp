#include "../include/husarion_bag_recorder/husarion_bag_recorder_node.hpp"

husarion_bag_recorder_node::husarion_bag_recorder_node(): Node("husarion_bag_recorder_node",
                                                        rclcpp::NodeOptions()
                                                    .allow_undeclared_parameters(true)
                                                    .automatically_declare_parameters_from_overrides(true))
{
    loadParams();
    setupConnections();
    _flags.saveToBag = -1;
    _flags.reduceOutliers = -1;
    _flags.allowRestart = true;
    //startRecording();
}

void husarion_bag_recorder_node::startRecording() {
    //RCLCPP_INFO(this->get_logger(), "before opening");

    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S", &tm);
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = this->get_parameter("file_path").as_string() + "/bag_" + std::string(buffer);
    storage_options.storage_id = "sqlite3";
    _bagWriter.open(storage_options);   
    //RCLCPP_INFO(this->get_logger(), "after opening");
    std::cout << "Local time: " << std::put_time(&tm, "%c %Z") << '\n';


}

void husarion_bag_recorder_node::stopRecording() {
    _bagWriter.close();
    
}
void husarion_bag_recorder_node::loadParams() {
    _topics.lidarTopic = this->get_parameter("lidar_subscriber_topic").as_string();
    _topics.imuLidarTopic = this->get_parameter("imu_lidar_subscriber_topic").as_string();
    _topics.vehicleOdometryTopic = this->get_parameter("vehicle_odom_subscriber_topic").as_string();
    _topics.joysticTopic = this->get_parameter("joystick_topic").as_string();
    _namespace = this->get_namespace();
}

void husarion_bag_recorder_node::setupConnections() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);



    _lidarSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
         _topics.lidarTopic, 10, std::bind(&husarion_bag_recorder_node::lidarCallback, this, std::placeholders::_1)); 
    _lidarImuSubscriber = this->create_subscription<sensor_msgs::msg::Imu>(
         _topics.imuLidarTopic, 10, std::bind(&husarion_bag_recorder_node::imuLidarCallback, this, std::placeholders::_1)); 

    _joystickSubscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        _topics.joysticTopic, qos, std::bind(&husarion_bag_recorder_node::joystickCallback, this, std::placeholders::_1));   
    _vehicleOdomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        _topics.vehicleOdometryTopic, qos, std::bind(&husarion_bag_recorder_node::vehicleOdomCallback, this, std::placeholders::_1));   
}

void husarion_bag_recorder_node::lidarCallback(std::shared_ptr<rclcpp::SerializedMessage> msg){
    rclcpp::Time time_stamp = this->now();
    if(_flags.saveToBag == 1)
        _bagWriter.write(msg, _namespace + "/" + _topics.lidarTopic, "sensor_msgs/msg/PointCloud2", time_stamp);
}

void husarion_bag_recorder_node::imuLidarCallback(std::shared_ptr<rclcpp::SerializedMessage> msg){
    rclcpp::Time time_stamp = this->now();
    if(_flags.saveToBag == 1)
        _bagWriter.write(msg, _namespace + "/" + _topics.imuLidarTopic, "sensor_msgs/msg/Imu", time_stamp);
}

void husarion_bag_recorder_node::vehicleOdomCallback(std::shared_ptr<rclcpp::SerializedMessage> msg){
    rclcpp::Time time_stamp = this->now();
    if(_flags.saveToBag == 1)
        _bagWriter.write(msg, _namespace + "/" + _topics.vehicleOdometryTopic, "nav_msgs/msg/Odometry", time_stamp);
}



void husarion_bag_recorder_node::joystickCallback(const sensor_msgs::msg::Joy &msg){
    if(_flags.allowRestart && msg.buttons[1] == 1){
        RCLCPP_INFO(this->get_logger(), "triggered start");
        _flags.saveToBag = 1;
        _flags.allowRestart = false;
        startRecording();

    }
    if(!_flags.allowRestart && msg.buttons[2] == 1) {
        RCLCPP_INFO(this->get_logger(), "triggered stop");
        _flags.allowRestart = true;
        _flags.saveToBag = false;
        stopRecording();
    }

}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<husarion_bag_recorder_node>());
    rclcpp::shutdown();
    return 0;
}