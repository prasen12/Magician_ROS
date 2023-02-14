#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "DobotDll.h"

/*
 * Cmd timeout
 */
#include "ros2_dobot/srv/set_cmd_timeout.hpp"

void SetCmdTimeoutService( std::shared_ptr<ros2_dobot::srv::SetCmdTimeout::Request> req, std::shared_ptr<ros2_dobot::srv::SetCmdTimeout::Response> res)
{
    res->result = SetCmdTimeout(req->timeout);
}


/*
 * Device information
 */
#include "ros2_dobot/srv/get_device_sn.hpp"
#include "ros2_dobot/srv/set_device_name.hpp"
#include "ros2_dobot/srv/get_device_name.hpp"
#include "ros2_dobot/srv/get_device_version.hpp"

void GetDeviceSNService(std::shared_ptr<ros2_dobot::srv::GetDeviceSN::Request> req, std::shared_ptr<ros2_dobot::srv::GetDeviceSN::Response> res)
{
    char deviceSN[256];

    res->result = GetDeviceSN(deviceSN, sizeof(deviceSN));
    if (res->result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceSN;
        res->device_sn = ss.str();
    }
}

void SetDeviceNameService(std::shared_ptr<ros2_dobot::srv::SetDeviceName::Request> req, std::shared_ptr<ros2_dobot::srv::SetDeviceName::Response> res)
{
    res->result = SetDeviceName(req->device_name.c_str());
}

void GetDeviceNameService(std::shared_ptr<ros2_dobot::srv::GetDeviceName::Request> req, std::shared_ptr<ros2_dobot::srv::GetDeviceName::Response> res)
{
    char deviceName[256];

    res->result = GetDeviceName(deviceName, sizeof(deviceName));
    if (res->result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceName; 
        res->device_name = ss.str();
    }
}

void GetDeviceVersionService(std::shared_ptr<ros2_dobot::srv::GetDeviceVersion::Request> req, std::shared_ptr<ros2_dobot::srv::GetDeviceVersion::Response> res) 
{
    uint8_t majorVersion, minorVersion, revision, hwRevision;

    res->result = GetDeviceVersion(&majorVersion, &minorVersion, &revision, &hwRevision);
    if (res->result == DobotCommunicate_NoError) {
        res->major_version = majorVersion;
        res->minor_version = minorVersion;
        res->revision = revision;
    }    
}

/*
 * Alarms
 */
#include "ros2_dobot/srv/get_alarms_state.hpp"
#include "ros2_dobot/srv/clear_all_alarms_state.hpp"

void GetAlarmsStateService(std::shared_ptr<ros2_dobot::srv::GetAlarmsState::Request> req, std::shared_ptr<ros2_dobot::srv::GetAlarmsState::Response> res)
{
    uint8_t alarms_state[128];
    uint32_t len;

    res->result = GetAlarmsState(alarms_state, &len, sizeof(alarms_state));
    if (res->result == DobotCommunicate_NoError) {
        for (int i = 0; i < len; i++) {
            res->alarms_state.push_back(alarms_state[i]);
        }
    }
}

void ClearAllAlarmsStateService(std::shared_ptr<ros2_dobot::srv::ClearAllAlarmsState::Request> req, std::shared_ptr<ros2_dobot::srv::ClearAllAlarmsState::Response> res)
{
    res->result = ClearAllAlarmsState();
}


/*
 * Pose
 */
#include "ros2_dobot/srv/get_pose.hpp"

void GetPoseService(std::shared_ptr<ros2_dobot::srv::GetPose::Request> req, std::shared_ptr<ros2_dobot::srv::GetPose::Response> res)
{
    Pose pose;

    res->result = GetPose(&pose);
    if (res->result == DobotCommunicate_NoError) {
        res->x = pose.x;
        res->y = pose.y;
        res->z = pose.z;
        res->r = pose.r;
        for (int i = 0; i < 4; i++) {
            res->joint_angle.push_back(pose.jointAngle[i]);
        }
    }
}


/*
 * HOME
 */
#include "ros2_dobot/srv/set_home_params.hpp"
#include "ros2_dobot/srv/get_home_params.hpp"
#include "ros2_dobot/srv/set_home_cmd.hpp"

void SetHOMEParamsService(std::shared_ptr<ros2_dobot::srv::SetHOMEParams::Request> req, std::shared_ptr<ros2_dobot::srv::SetHOMEParams::Response> res)
{
    HOMEParams params;
    uint64_t queuedCmdIndex;

    params.x = req->x;
    params.y = req->y;
    params.z = req->z;
    params.r = req->r;

    res->result = SetHOMEParams(&params, req->is_queued, &queuedCmdIndex);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queuedCmdIndex;
    }
}

void GetHOMEParamsService(std::shared_ptr<ros2_dobot::srv::GetHOMEParams::Request> req, std::shared_ptr<ros2_dobot::srv::GetHOMEParams::Response> res)
{
    HOMEParams params;

    res->result = GetHOMEParams(&params);
    if (res->result == DobotCommunicate_NoError) {
        res->x = params.x;
        res->y = params.y;
        res->z = params.z;
        res->r = params.r;
    }
}

void SetHOMECmdService(std::shared_ptr<ros2_dobot::srv::SetHOMECmd::Request> req, std::shared_ptr<ros2_dobot::srv::SetHOMECmd::Response> res)
{
    HOMECmd cmd;
    uint64_t queuedCmdIndex;

    res->result = SetHOMECmd(&cmd, true, &queuedCmdIndex);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queuedCmdIndex;
    }
}


/**
 * Main
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("DobotServer");

    if (argc < 2) {
        RCLCPP_ERROR(node->get_logger(), "Usage:  <Application> <portName>");
        return -1;
    }
    // Connect Dobot before start the service
    int result = ConnectDobot(argv[1], 115200, 0, 0, 0);
    switch (result) {
        case DobotConnect_NoError:
        break;
        case DobotConnect_NotFound:
            RCLCPP_ERROR(node->get_logger(), "Dobot not found!");
            return -2;
        break;
        case DobotConnect_Occupied:
            RCLCPP_ERROR(node->get_logger(), "Invalid port name or Dobot is being used by another application!");
            return -3;
        break;
        default:
        break;
    }

    RCLCPP_INFO(node->get_logger(), "Setting up Dobot services ...");
    auto cmdTimeoutService          = node->create_service<ros2_dobot::srv::SetCmdTimeout>("/DobotServer/SetCmdTimeout", &SetCmdTimeoutService);
    auto getDeviceSMService         = node->create_service<ros2_dobot::srv::GetDeviceSN>("/DobotServer/GetDeviceSN", &GetDeviceSNService);
    auto setDeviceNameService       = node->create_service<ros2_dobot::srv::SetDeviceName>("/DobotServer/SetDeviceName", &SetDeviceNameService);
    auto getDeviceNameService       = node->create_service<ros2_dobot::srv::GetDeviceName>("/DobotServer/GetDeviceName", &GetDeviceNameService);
    auto getDeviceVersionSerivce    = node->create_service<ros2_dobot::srv::GetDeviceVersion>("/DobotServer/GetDeviceVersion", &GetDeviceVersionService);
    auto getAlarmStateService       = node->create_service<ros2_dobot::srv::GetAlarmsState>("/DobotServer/GetAlarmsState", &GetAlarmsStateService);
    auto clearAllAlarmsStateService = node->create_service<ros2_dobot::srv::ClearAllAlarmsState>("/DobotServer/ClearAllAlarmsState", &ClearAllAlarmsStateService);
    auto getPoseService             = node->create_service<ros2_dobot::srv::GetPose>("/DobotServer/GetPoseService", &GetPoseService);
    auto setHOMEParamsService       = node->create_service<ros2_dobot::srv::SetHOMEParams>("/DobotServer/SetHOMEParams", &SetHOMEParamsService);
    auto getHOMEParamsService       = node->create_service<ros2_dobot::srv::GetHOMEParams>("/DobotServer/GetHOMEParams", &GetHOMEParamsService);
    auto setHOMECmdService          = node->create_service<ros2_dobot::srv::SetHOMECmd>("/DobotServer/SetHOMECmd", &SetHOMECmdService);


    RCLCPP_INFO(node->get_logger(), "Dobot Server Node running...");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Dobot Server Node exiting...");
    
    // Disconnect Dobot
    DisconnectDobot();
    rclcpp::shutdown();
}

