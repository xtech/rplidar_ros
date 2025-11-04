/*
 *  RPLIDAR ROS2 NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/empty.hpp>
#include "sl_lidar.h"
#include "math.h"

#include <signal.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

#define ROS2VERSION "1.0.1"

enum {
    LIDAR_A_SERIES_MINUM_MAJOR_ID   = 0,
    LIDAR_S_SERIES_MINUM_MAJOR_ID   = 5,
    LIDAR_T_SERIES_MINUM_MAJOR_ID   = 8,
};

using namespace sl;

bool need_exit = false;

class RPlidarNode : public rclcpp::Node
{
  public:
    RPlidarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("rplidar_node", options)
    {
        // Register dynamic parameter callback to allow runtime updates
        param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto &p : params) {
                    if (p.get_name() == "time_offset") {
                        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                            double new_val = p.as_double();
                            time_offset_ms = new_val;
                            RCLCPP_INFO(this->get_logger(), "Updated time_offset to %.3f ms at runtime", time_offset_ms);
                        } else {
                            result.successful = false;
                            result.reason = "time_offset must be a double";
                            break;
                        }
                    } else if (p.get_name() == "time_increment_multiplier") {
                        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                            double new_val = p.as_double();
                            time_increment_multiplier = new_val;
                            RCLCPP_INFO(this->get_logger(), "Updated time_increment_multiplier to %.3f ms at runtime", time_increment_multiplier);
                        } else {
                            result.successful = false;
                            result.reason = "time_increment_multiplier must be a double";
                            break;
                        }
                    } else if (p.get_name() == "output_points") {
                        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                            int new_val = p.as_int();

                            if (new_val <= 0) {
                                result.successful = false;
                                result.reason = "Output point count must be positive";
                                break;
                            }

                            output_points = new_val;
                            RCLCPP_INFO(this->get_logger(), "Updated output_points to %d at runtime", output_points);
                        } else {
                            result.successful = false;
                            result.reason = "output_points must be an integer";
                        }
                    }
                }
                return result;
            }
        );
    }

  private:    
    void init_param()
    {
        this->declare_parameter<std::string>("channel_type","serial");
        this->declare_parameter<std::string>("tcp_ip", "192.168.0.7");
        this->declare_parameter<int>("tcp_port", 20108);
        this->declare_parameter<std::string>("udp_ip","192.168.11.2");
        this->declare_parameter<int>("udp_port",8089);
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("serial_baudrate",1000000);
        this->declare_parameter<std::string>("frame_id","laser_frame");
        this->declare_parameter<bool>("inverted", false);
        this->declare_parameter<bool>("flip_x_axis", false);
        this->declare_parameter<bool>("auto_standby", false);
        this->declare_parameter<bool>("publish_cloud", false);
        this->declare_parameter<std::string>("topic_name",std::string("scan"));
        this->declare_parameter<std::string>("scan_mode",std::string());
        this->declare_parameter<float>("scan_frequency",10);
        this->declare_parameter<double>("time_offset", 0.0);
        this->declare_parameter<double>("time_increment_multiplier", 1.0);
        this->declare_parameter<int>("output_points", 360);

        this->get_parameter_or<std::string>("channel_type", channel_type, "serial");
        this->get_parameter_or<std::string>("tcp_ip", tcp_ip, "192.168.0.7"); 
        this->get_parameter_or<int>("tcp_port", tcp_port, 20108);
        this->get_parameter_or<std::string>("udp_ip", udp_ip, "192.168.11.2"); 
        this->get_parameter_or<int>("udp_port", udp_port, 8089);
        this->get_parameter_or<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
        this->get_parameter_or<int>("serial_baudrate", serial_baudrate, 1000000/*256000*/);//ros run for A1 A2, change to 256000 if A3
        this->get_parameter_or<std::string>("frame_id", frame_id, "laser_frame");
        this->get_parameter_or<bool>("inverted", inverted, false);
        this->get_parameter_or<bool>("flip_x_axis", flip_x_axis, false);
        this->get_parameter_or<bool>("publish_cloud", publish_cloud, false);
        this->get_parameter_or<bool>("auto_standby", auto_standby, false);
        this->get_parameter_or<std::string>("topic_name", topic_name, "scan");
        this->get_parameter_or<std::string>("scan_mode", scan_mode, std::string());
        if(channel_type == "udp")
            this->get_parameter_or<float>("scan_frequency", scan_frequency, 20.0);
        else
            this->get_parameter_or<float>("scan_frequency", scan_frequency, 10.0);

        this->get_parameter_or<int>("output_points", output_points, 360);
        this->get_parameter_or<double>("time_offset", time_offset_ms, 0.0);
        this->get_parameter_or<double>("time_increment_multiplier", time_increment_multiplier, 1.0);
    }

    bool getRPLIDARDeviceInfo(ILidarDriver * drv)
    {
        sl_result     op_result;
        sl_lidar_response_device_info_t devinfo;

        op_result = drv->getDeviceInfo(devinfo);
        if (SL_IS_FAIL(op_result)) {
            if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
                RCLCPP_ERROR(this->get_logger(),"Error, operation time out. SL_RESULT_OPERATION_TIMEOUT! ");
            } else {
                RCLCPP_ERROR(this->get_logger(),"Error, unexpected error, code: %x",op_result);
            }
            return false;
        }

        // print out the device serial number, firmware and hardware version number..
        char sn_str[37] = {'\0'}; 
        for (int pos = 0; pos < 16 ;++pos) {
            sprintf(sn_str + (pos * 2),"%02X", devinfo.serialnum[pos]);
        }
        RCLCPP_INFO(this->get_logger(),"RPLidar S/N: %s",sn_str);
        RCLCPP_INFO(this->get_logger(),"Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
        RCLCPP_INFO(this->get_logger(),"Hardware Rev: %d",(int)devinfo.hardware_version);
        return true;
    }

    bool checkRPLIDARHealth(ILidarDriver * drv)
    {
        sl_result     op_result;
        sl_lidar_response_device_health_t healthinfo;
        op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result)) { 
            RCLCPP_INFO(this->get_logger(),"RPLidar health status : %d", healthinfo.status);
            switch (healthinfo.status) {
                case SL_LIDAR_STATUS_OK:
                    RCLCPP_INFO(this->get_logger(),"RPLidar health status : OK.");
                    return true;
                case SL_LIDAR_STATUS_WARNING:
                    RCLCPP_INFO(this->get_logger(),"RPLidar health status : Warning.");
                    return true;
                case SL_LIDAR_STATUS_ERROR:
                    RCLCPP_ERROR(this->get_logger(),"Error, RPLidar internal error detected. Please reboot the device to retry.");
                    return false;
                default:
                    RCLCPP_ERROR(this->get_logger(),"Error, Unknown internal error detected. Please reboot the device to retry.");
                    return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),"Error, cannot retrieve RPLidar health code: %x", op_result);
            return false;
        }
    }

    bool stop_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if (auto_standby) {
            RCLCPP_INFO(
                this->get_logger(),
                "Ingnoring stop_motor request because rplidar_node is in 'auto standby' mode");
            return false;
        }

        RCLCPP_DEBUG(this->get_logger(), "Call to '%s'", __FUNCTION__);
        
        //RCLCPP_DEBUG(this->get_logger(),"Stop motor");
        this->stop();
        //drv->setMotorSpeed(0);
        return true;
    }

    bool start_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if (auto_standby) {
            RCLCPP_INFO(
                this->get_logger(),
                "Ingnoring start_motor request because rplidar_node is in 'auto standby' mode");
            return false;
        }
        RCLCPP_DEBUG(this->get_logger(), "Call to '%s'", __FUNCTION__);
        return this->start();

#if 0
        if(!drv)
           return false;
        if(drv->isConnected())
        {
            RCLCPP_DEBUG(this->get_logger(),"Start motor");
            sl_result ans=drv->setMotorSpeed();
            if (SL_IS_FAIL(ans)) {
                RCLCPP_WARN(this->get_logger(), "Failed to start motor: %08x", ans);
                return false;
            }
        
            ans=drv->startScan(0,1);
            if (SL_IS_FAIL(ans)) {
                RCLCPP_WARN(this->get_logger(), "Failed to start scan: %08x", ans);
            }
        } else {
            RCLCPP_INFO(this->get_logger(),"lost connection");
            return false;
        }

        return true;
#endif

    }

    static float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return (node.angle_z_q14 * 90.f / 16384.f) * M_PI / 180.f;
    }


    bool set_scan_mode() {
        sl_result     op_result;
        if (scan_mode.empty()) {
            op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
        }
        else {
            std::vector<LidarScanMode> allSupportedScanModes;
            op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

            if (SL_IS_OK(op_result)) {
                sl_u16 selectedScanMode = sl_u16(-1);
                for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    if (iter->scan_mode == scan_mode) {
                        selectedScanMode = iter->id;
                        break;
                    }
                }

                if (selectedScanMode == sl_u16(-1)) {
                    RCLCPP_ERROR(this->get_logger(), "scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                    for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                        RCLCPP_ERROR(this->get_logger(), "\t%s: max_distance: %.1f m, Point number: %.1fK", iter->scan_mode,
                            iter->max_distance, (1000 / iter->us_per_sample));
                    }
                    op_result = SL_RESULT_OPERATION_FAIL;
                }
                else {
                    op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
                }
            }
        }

        if (SL_IS_OK(op_result))
        {
            max_distance = (float)current_scan_mode.max_distance;
            RCLCPP_INFO(this->get_logger(), "current scan mode: %s, sample time: %d uS, max_distance: %.1f m, scan frequency:%.1f Hz, ",
                current_scan_mode.scan_mode, (int)(current_scan_mode.us_per_sample), max_distance, scan_frequency);
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Can not start scan: %08x!", op_result);
            return false;
        }
    }
    bool start()
    {
        if (nullptr == drv) {
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Start");
        drv->setMotorSpeed();
        if (!set_scan_mode()) {
            this->stop();
            RCLCPP_ERROR(this->get_logger(), "Failed to set scan mode");
            return false;
        }
        is_scanning = true;
        return true;
    }

    void stop()
    {
        if (nullptr == drv) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Stop");
        drv->stop();
        drv->setMotorSpeed(0);
        is_scanning = false;
    }

public:    
    int work_loop()
    {        
        init_param();
        int ver_major = SL_LIDAR_SDK_VERSION_MAJOR;
        int ver_minor = SL_LIDAR_SDK_VERSION_MINOR;
        int ver_patch = SL_LIDAR_SDK_VERSION_PATCH;
        RCLCPP_INFO(this->get_logger(),"RPLidar running on ROS2 package rplidar_ros. RPLIDAR SDK Version:%d.%d.%d",ver_major,ver_minor,ver_patch);
    
        sl_result     op_result;
        // create the driver instance
        drv = *createLidarDriver();
        if (nullptr == drv) {
            /* don't start spinning without a driver object */
            RCLCPP_ERROR(this->get_logger(), "Failed to construct driver");
            return -1;
        }
        IChannel* _channel;
        if(channel_type == "tcp"){
            _channel = *createTcpChannel(tcp_ip, tcp_port);
        }
        else if(channel_type == "udp"){
            _channel = *createUdpChannel(udp_ip, udp_port);
        }
        else{
            _channel = *createSerialPortChannel(serial_port, serial_baudrate);
        }
        if (SL_IS_FAIL((drv)->connect(_channel))) {
            if(channel_type == "tcp"){
                RCLCPP_ERROR(this->get_logger(),"Error, cannot connect to the ip addr  %s with the tcp port %s.",tcp_ip.c_str(),std::to_string(tcp_port).c_str());
            }
            else if(channel_type == "udp"){
                RCLCPP_ERROR(this->get_logger(),"Error, cannot connect to the ip addr  %s with the udp port %s.",udp_ip.c_str(),std::to_string(udp_port).c_str());
            }
            else{
                RCLCPP_ERROR(this->get_logger(),"Error, cannot bind to the specified serial port %s.",serial_port.c_str());            
            }
            delete drv; drv = nullptr;
            return -1;
        }
        
        // get rplidar device info
        if (!getRPLIDARDeviceInfo(drv)) {
            delete drv; drv = nullptr;
            return -1;
        }

        // check health...
        if (!checkRPLIDARHealth(drv)) {
            delete drv; drv = nullptr;
            return -1;
        }

        sl_lidar_response_device_info_t devinfo;
        op_result = drv->getDeviceInfo(devinfo);
        bool scan_frequency_tunning_after_scan = false;

        if( (devinfo.model>>4) > LIDAR_S_SERIES_MINUM_MAJOR_ID){
            scan_frequency_tunning_after_scan = true;
        }

        if(!scan_frequency_tunning_after_scan){ //for RPLIDAR A serials
            //start RPLIDAR A serials  rotate by pwm
            drv->setMotorSpeed(600);
        }

        /* start motor and scanning */
        if (!auto_standby && !this->start()) {
            delete drv; drv = nullptr;
            return -1;
        }

        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, rclcpp::QoS(rclcpp::KeepLast(10)));
        if (publish_cloud) {
            cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::QoS(rclcpp::KeepLast(10)));
        } else {
            cloud_pub = nullptr;
        }

        imu_sub =   this->create_subscription<sensor_msgs::msg::Imu>(
                        "imu", rclcpp::SensorDataQoS(), std::bind(&RPlidarNode::on_imu, this, std::placeholders::_1));

        stop_motor_service = this->create_service<std_srvs::srv::Empty>("stop_motor",  
                                std::bind(&RPlidarNode::stop_motor,this,std::placeholders::_1,std::placeholders::_2));
        start_motor_service = this->create_service<std_srvs::srv::Empty>("start_motor", 
                                std::bind(&RPlidarNode::start_motor,this,std::placeholders::_1,std::placeholders::_2));

        //drv->setMotorSpeed();

        rclcpp::Time start_scan_time;
        rclcpp::Time end_scan_time;
        double scan_duration;
        sensor_msgs::msg::LaserScan laser_msg;
        laser_msg.header.frame_id = frame_id;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        if (cloud_pub != nullptr) {
            cloud_msg.header.frame_id = frame_id;
            cloud_msg.fields.resize(4);
            // x
            cloud_msg.fields[0].name = "x";
            cloud_msg.fields[0].offset = 0;
            cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud_msg.fields[0].count = 1;

            // y
            cloud_msg.fields[1].name = "y";
            cloud_msg.fields[1].offset = 4;
            cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud_msg.fields[1].count = 1;

            // z
            cloud_msg.fields[2].name = "z";
            cloud_msg.fields[2].offset = 8;
            cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud_msg.fields[2].count = 1;

            // intensity
            cloud_msg.fields[3].name = "intensity";
            cloud_msg.fields[3].offset = 12;
            cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud_msg.fields[3].count = 1;


            cloud_msg.is_bigendian = false;
            cloud_msg.point_step = 16;
            cloud_msg.height = 1;
            cloud_msg.width = 0;
        }
        while (rclcpp::ok() && !need_exit) {
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t   count = _countof(nodes);

            if (auto_standby) {
                if (scan_pub->get_subscription_count() > 0 && !is_scanning) {
                    this->start();
                }
                else if (scan_pub->get_subscription_count() == 0) {
                    if (is_scanning) {
                        this->stop();
                    }
                }
            }

            start_scan_time = this->now();
            op_result = drv->grabScanDataHq(nodes, count);

            // Apply time offset (milliseconds) to start and end timestamps
            rclcpp::Duration time_offset = rclcpp::Duration::from_nanoseconds((int64_t)(time_offset_ms * 1e6));
            rclcpp::Time start_scan_time_adj = start_scan_time + time_offset;


            bool has_cloud_subscriber = cloud_pub != nullptr && cloud_pub->get_subscription_count() > 0;
            bool has_scan_subscriber = scan_pub->get_subscription_count() > 0;


            if (op_result == SL_RESULT_OK) {
                if(scan_frequency_tunning_after_scan) { //Set scan frequency(For Slamtec Tof lidar)
                    RCLCPP_INFO(this->get_logger(), "set lidar scan frequency to %.1f Hz(%.1f Rpm) ",scan_frequency,scan_frequency*60);
                    drv->setMotorSpeed(scan_frequency*60); //rpm 
                    scan_frequency_tunning_after_scan = false;
                    continue;
                }

                // reserve enough space for the output points
                laser_msg.ranges.resize(output_points);

                // Zero the vector, since we might not have data for each bucket
                memset(laser_msg.ranges.data(), 0, laser_msg.ranges.size() * sizeof(float));

                size_t cloud_write_index = 0;
                // reserve enough space in case all measurements are valid
                if (has_cloud_subscriber) {
                    cloud_msg.data.resize(count * 4 * sizeof(float));
                }
                static_assert(sizeof(float) == 4, "float must be 4 bytes");
                for (size_t i = 0; i < count; i++) {
                    float angle = getAngle(nodes[i]);
                    float range = nodes[i].dist_mm_q2 / 4000.f;
                    // account for rotation using IMU data
                    float time_since_scan_start = (float)(current_scan_mode.us_per_sample * i) / 1000000.0f;
                    angle -= time_since_scan_start * static_cast<float>(last_imu.angular_velocity.z) * time_increment_multiplier;

                    // Write to cloud
                    if (range > 0 && range < max_distance) {
                        if (has_cloud_subscriber) {
                            float *p = reinterpret_cast<float *>(&cloud_msg.data[cloud_write_index]);
                            p[0] = cos(angle) * range;
                            p[1] = -sin(angle) * range;
                            p[2] = 0.0f;
                            p[3] = nodes[i].quality;
                            // wrote a value, advance the index
                            cloud_write_index+=4 * sizeof(float);
                        }

                        if (has_scan_subscriber) {
                            // find the bucket
                            angle = fmodf(angle, 2.0f * M_PI);
                            while (angle < 0.0f) {
                                angle += 2.0f * M_PI;
                            }
                            ssize_t bucket_index = (ssize_t)((((2.0*M_PI) - angle) * laser_msg.ranges.size()) / (2.0*M_PI));
                            if (bucket_index >= 0 && bucket_index < laser_msg.ranges.size()) {
                                laser_msg.ranges[bucket_index] = range;
                            } else {
                                RCLCPP_WARN(this->get_logger(), "Invalid bucket index: %d", bucket_index);
                            }
                        }
                    }
                }

                if (cloud_write_index > 0) {
                    // shrink in case not all measurements were valid
                    cloud_msg.data.resize(cloud_write_index);
                    cloud_msg.width = cloud_write_index/cloud_msg.point_step;
                    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
                    cloud_msg.header.stamp = start_scan_time_adj;
                    cloud_pub->publish(cloud_msg);
                }


                if (has_scan_subscriber) {
                    laser_msg.header.stamp = start_scan_time_adj;
                    laser_msg.angle_min = 0.0;
                    laser_msg.angle_max = 2.0* M_PI;
                    laser_msg.angle_increment = (2.0* M_PI) / output_points;
                    // we have already deskewed, so simulate that its an instant scan
                    laser_msg.time_increment = 0.0;
                    laser_msg.scan_time = 0.0;
                    laser_msg.range_min = 0.0;
                    laser_msg.range_max = max_distance;
                    scan_pub->publish(laser_msg);
                }
            }

            rclcpp::spin_some(shared_from_this());
        }

        // done!
        drv->setMotorSpeed(0);
        drv->stop();
        RCLCPP_INFO(this->get_logger(),"Stop motor");
        if (drv) { delete drv;  drv = nullptr; }
        return 0;
    }

    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu = *msg;
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service;

    std::string channel_type;
    std::string tcp_ip;
    std::string udp_ip;
    std::string serial_port;
    std::string topic_name;
    int tcp_port = 20108;
    int udp_port = 8089;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool flip_x_axis = false;
    bool publish_cloud = false;
    bool auto_standby = false;
    float max_distance = 8.0;
    std::string scan_mode;
    float scan_frequency;
    int output_points = 360;
    double time_offset_ms = 0.0; // timestamp offset applied to start and end times
    double time_increment_multiplier = 1.0; // multiplier applied to time increment, for debugging deskew timing issues
    /* State */
    bool is_scanning = false;
    LidarScanMode current_scan_mode{};
    ILidarDriver *drv = nullptr;
    sensor_msgs::msg::Imu last_imu{};

    // Keep the parameter callback handle alive
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

void ExitHandler(int sig)
{
    (void)sig;
    need_exit = true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  
  auto rplidar_node = std::make_shared<RPlidarNode>(rclcpp::NodeOptions());
  signal(SIGINT,ExitHandler);
  int ret = rplidar_node->work_loop();
  rclcpp::shutdown();
  return ret;
}

