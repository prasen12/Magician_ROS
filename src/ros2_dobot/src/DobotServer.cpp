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


/*
 * Queued command control
 */
#include "ros2_dobot/srv/set_queued_cmd_start_exec.hpp"
#include "ros2_dobot/srv/set_queued_cmd_stop_exec.hpp"
#include "ros2_dobot/srv/set_queued_cmd_force_stop_exec.hpp"
#include "ros2_dobot/srv/set_queued_cmd_clear.hpp"

void SetQueuedCmdStartExecService(std::shared_ptr<ros2_dobot::srv::SetQueuedCmdStartExec::Request> req, std::shared_ptr<ros2_dobot::srv::SetQueuedCmdStartExec::Response> res)
{
    res->result = SetQueuedCmdStartExec();
}

void SetQueuedCmdStopExecService(std::shared_ptr<ros2_dobot::srv::SetQueuedCmdStopExec::Request> req, std::shared_ptr<ros2_dobot::srv::SetQueuedCmdStopExec::Response> res)
{
    res->result = SetQueuedCmdStopExec();
}

void SetQueuedCmdForceStopExecService(std::shared_ptr<ros2_dobot::srv::SetQueuedCmdForceStopExec::Request> req, std::shared_ptr<ros2_dobot::srv::SetQueuedCmdForceStopExec::Response> res)
{
    res->result = SetQueuedCmdForceStopExec();
}

void SetQueuedCmdClearService(std::shared_ptr<ros2_dobot::srv::SetQueuedCmdClear::Request> req, std::shared_ptr<ros2_dobot::srv::SetQueuedCmdClear::Response> res)
{
    res->result = SetQueuedCmdClear();
}

/*
 * End effector
 */
#include "ros2_dobot/srv/set_end_effector_params.hpp"
#include "ros2_dobot/srv/get_end_effector_params.hpp"
#include "ros2_dobot/srv/set_end_effector_laser.hpp"
#include "ros2_dobot/srv/get_end_effector_laser.hpp"
#include "ros2_dobot/srv/set_end_effector_suction_cup.hpp"
#include "ros2_dobot/srv/get_end_effector_suction_cup.hpp"
#include "ros2_dobot/srv/set_end_effector_gripper.hpp"
#include "ros2_dobot/srv/get_end_effector_gripper.hpp"

void SetEndEffectorParamsService(std::shared_ptr<ros2_dobot::srv::SetEndEffectorParams::Request> req, std::shared_ptr<ros2_dobot::srv::SetEndEffectorParams::Response> res)
{
    EndEffectorParams params;
    uint64_t queued_cmd_index;
    
    params.xBias = req->x_bias;
    params.yBias = req->y_bias;
    params.zBias = req->z_bias;

    res->result = SetEndEffectorParams(&params, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }
}   
    
void GetEndEffectorParamsService(std::shared_ptr<ros2_dobot::srv::GetEndEffectorParams::Request> req, std::shared_ptr<ros2_dobot::srv::GetEndEffectorParams::Response> res)
{
    EndEffectorParams params;

    res->result = GetEndEffectorParams(&params);
    if (res->result == DobotCommunicate_NoError) {
        res->x_bias = params.xBias;
        res->y_bias = params.yBias;
        res->z_bias = params.zBias;
    }
}

bool SetEndEffectorLaserService(std::shared_ptr<ros2_dobot::srv::SetEndEffectorLaser::Request> req, std::shared_ptr<ros2_dobot::srv::SetEndEffectorLaser::Response> res)
{
    uint64_t queued_cmd_index;

    res->result = SetEndEffectorLaser(req->enable_ctrl, req->on, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

    return true;
}

bool GetEndEffectorLaserService(std::shared_ptr<ros2_dobot::srv::GetEndEffectorLaser::Request> req, std::shared_ptr<ros2_dobot::srv::GetEndEffectorLaser::Response> res)
{
    bool enable_ctrl, on;

    res->result = GetEndEffectorLaser(&enable_ctrl, &on);
    if (res->result == DobotCommunicate_NoError) {
        res->enable_ctrl = enable_ctrl;
        res->on = on;
    }

    return true;
}

bool SetEndEffectorSuctionCupService(std::shared_ptr<ros2_dobot::srv::SetEndEffectorSuctionCup::Request> req, std::shared_ptr<ros2_dobot::srv::SetEndEffectorSuctionCup::Response> res)
{
    uint64_t queued_cmd_index;

    res->result = SetEndEffectorSuctionCup(req->enable_ctrl, req->suck, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

    return true;
}
bool GetEndEffectorSuctionCupService(std::shared_ptr<ros2_dobot::srv::GetEndEffectorSuctionCup::Request> req, std::shared_ptr<ros2_dobot::srv::GetEndEffectorSuctionCup::Response> res)
{
    bool enable_ctrl, suck;

    res->result = GetEndEffectorLaser(&enable_ctrl, &suck);
    if (res->result == DobotCommunicate_NoError) {
        res->enable_ctrl = enable_ctrl;
        res->suck = suck;
    }

    return true;
}

bool SetEndEffectorGripperService(std::shared_ptr<ros2_dobot::srv::SetEndEffectorGripper::Request> req, std::shared_ptr<ros2_dobot::srv::SetEndEffectorGripper::Response> res)
{
    uint64_t queued_cmd_index;

    res->result = SetEndEffectorGripper(req->enable_ctrl, req->grip, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

    return true;
}

bool GetEndEffectorGripperService(std::shared_ptr<ros2_dobot::srv::GetEndEffectorGripper::Request> req, std::shared_ptr<ros2_dobot::srv::GetEndEffectorGripper::Response> res)
{
    bool enable_ctrl, grip;

    res->result = GetEndEffectorLaser(&enable_ctrl, &grip);
    if (res->result == DobotCommunicate_NoError) {
        res->enable_ctrl = enable_ctrl;
        res->grip = grip;
    }

    return true;
}


/*
 * PTP
 */
#include "ros2_dobot/srv/set_ptp_joint_params.hpp"
#include "ros2_dobot/srv/get_ptp_joint_params.hpp"
#include "ros2_dobot/srv/set_ptp_coordinate_params.hpp"
#include "ros2_dobot/srv/get_ptp_coordinate_params.hpp"
#include "ros2_dobot/srv/set_ptp_jump_params.hpp"
#include "ros2_dobot/srv/get_ptp_jump_params.hpp"
#include "ros2_dobot/srv/set_ptp_common_params.hpp"
#include "ros2_dobot/srv/get_ptp_common_params.hpp"
#include "ros2_dobot/srv/set_ptp_cmd.hpp"
    
void SetPTPJointParamsService(std::shared_ptr<ros2_dobot::srv::SetPTPJointParams::Request> req, std::shared_ptr<ros2_dobot::srv::SetPTPJointParams::Response> res)
{   
    PTPJointParams params;
    uint64_t queued_cmd_index;
    
    for (int i = 0; i < req->velocity.size(); i++) {
        params.velocity[i] = req->velocity[i];
    }
    for (int i = 0; i < req->acceleration.size(); i++) {
        params.acceleration[i] = req->acceleration[i];
    }
    res->result = SetPTPJointParams(&params, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }
}

void GetPTPJointParamsService(std::shared_ptr<ros2_dobot::srv::GetPTPJointParams::Request> req, std::shared_ptr<ros2_dobot::srv::GetPTPJointParams::Response> res)
{
    PTPJointParams params;

    res->result = GetPTPJointParams(&params);
    if (res->result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res->velocity.push_back(params.velocity[i]);
            res->acceleration.push_back(params.acceleration[i]);
        }
    }
}

void SetPTPCoordinateParamsService(std::shared_ptr<ros2_dobot::srv::SetPTPCoordinateParams::Request> req, std::shared_ptr<ros2_dobot::srv::SetPTPCoordinateParams::Response> res)
{
    PTPCoordinateParams params;
    uint64_t queued_cmd_index;

    params.xyzVelocity = req->xyz_velocity;
    params.rVelocity = req->r_velocity;
    params.xyzAcceleration = req->xyz_acceleration;
    params.rAcceleration = req->r_acceleration;
    res->result = SetPTPCoordinateParams(&params, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }
}

void GetPTPCoordinateParamsService(std::shared_ptr<ros2_dobot::srv::GetPTPCoordinateParams::Request> req, std::shared_ptr<ros2_dobot::srv::GetPTPCoordinateParams::Response> res)
{
    PTPCoordinateParams params;

    res->result = GetPTPCoordinateParams(&params);
    if (res->result == DobotCommunicate_NoError) {
        res->xyz_velocity = params.xyzVelocity;
        res->r_velocity = params.rVelocity;
        res->xyz_acceleration = params.xyzAcceleration;
        res->r_acceleration = params.rAcceleration;
    }
}

void SetPTPJumpParamsService(std::shared_ptr<ros2_dobot::srv::SetPTPJumpParams::Request> req, std::shared_ptr<ros2_dobot::srv::SetPTPJumpParams::Response> res)
{
    PTPJumpParams params;
    uint64_t queued_cmd_index;

    params.jumpHeight = req->jump_height;
    params.zLimit = req->z_limit;
    res->result = SetPTPJumpParams(&params, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }
}

void GetPTPJumpParamsService(std::shared_ptr<ros2_dobot::srv::GetPTPJumpParams::Request> req, std::shared_ptr<ros2_dobot::srv::GetPTPJumpParams::Response> res)
{
    PTPJumpParams params;

    res->result = GetPTPJumpParams(&params);
    if (res->result == DobotCommunicate_NoError) {
        res->jump_height = params.jumpHeight;
        res->z_limit = params.zLimit;
    }
}

void SetPTPCommonParamsService(std::shared_ptr<ros2_dobot::srv::SetPTPCommonParams::Request> req, std::shared_ptr<ros2_dobot::srv::SetPTPCommonParams::Response> res)
{
    PTPCommonParams params;
    uint64_t queued_cmd_index;

    params.velocityRatio = req->velocity_ratio;
    params.accelerationRatio = req->acceleration_ratio;
    res->result = SetPTPCommonParams(&params, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }
}

void SetPTPCmdService(std::shared_ptr<ros2_dobot::srv::SetPTPCmd::Request> req, std::shared_ptr<ros2_dobot::srv::SetPTPCmd::Response> res)
{
    PTPCmd cmd;
    uint64_t queued_cmd_index;

    cmd.ptpMode = req->ptp_mode;
    cmd.x = req->x;
    cmd.y = req->y;
    cmd.z = req->z;
    cmd.r = req->r;
    res->result = SetPTPCmd(&cmd, true, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}


void GetPTPCommonParamsService(std::shared_ptr<ros2_dobot::srv::GetPTPCommonParams::Request> req, std::shared_ptr<ros2_dobot::srv::GetPTPCommonParams::Response> res)
{
    PTPCommonParams params;

    res->result = GetPTPCommonParams(&params);
    if (res->result == DobotCommunicate_NoError) {
        res->velocity_ratio = params.velocityRatio;
        res->acceleration_ratio = params.accelerationRatio;
    }
}




/**
 * Create services
 */
void CreateServices(std::shared_ptr<rclcpp::Node> node) {
    node->create_service<ros2_dobot::srv::SetCmdTimeout>("/DobotServer/SetCmdTimeout", &SetCmdTimeoutService);

    // Device Information Services
    node->create_service<ros2_dobot::srv::GetDeviceSN>("/DobotServer/GetDeviceSN", &GetDeviceSNService);
    node->create_service<ros2_dobot::srv::SetDeviceName>("/DobotServer/SetDeviceName", &SetDeviceNameService);
    node->create_service<ros2_dobot::srv::GetDeviceName>("/DobotServer/GetDeviceName", &GetDeviceNameService);
    node->create_service<ros2_dobot::srv::GetDeviceVersion>("/DobotServer/GetDeviceVersion", &GetDeviceVersionService);

    // Alarm Services
    node->create_service<ros2_dobot::srv::GetAlarmsState>("/DobotServer/GetAlarmsState", &GetAlarmsStateService);
    node->create_service<ros2_dobot::srv::ClearAllAlarmsState>("/DobotServer/ClearAllAlarmsState", &ClearAllAlarmsStateService);

    // Pose Services
    node->create_service<ros2_dobot::srv::GetPose>("/DobotServer/GetPoseService", &GetPoseService);
    node->create_service<ros2_dobot::srv::SetHOMEParams>("/DobotServer/SetHOMEParams", &SetHOMEParamsService);
    node->create_service<ros2_dobot::srv::GetHOMEParams>("/DobotServer/GetHOMEParams", &GetHOMEParamsService);
    node->create_service<ros2_dobot::srv::SetHOMECmd>("/DobotServer/SetHOMECmd", &SetHOMECmdService);

    // Queue Command Services
    node->create_service<ros2_dobot::srv::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear", &SetQueuedCmdClearService);
    node->create_service<ros2_dobot::srv::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec", &SetQueuedCmdStartExecService);
    node->create_service<ros2_dobot::srv::SetQueuedCmdStopExec>("/DobotServer/SetQueuedCmdStopExecService", &SetQueuedCmdStopExecService);
    node->create_service<ros2_dobot::srv::SetQueuedCmdForceStopExec>("/DobotServer/SetQueuedCmdForceStopExecService", &SetQueuedCmdForceStopExecService);

    // End Effector Services
    node->create_service<ros2_dobot::srv::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams", &SetEndEffectorParamsService);
    node->create_service<ros2_dobot::srv::GetEndEffectorParams>("/DobotServer/GetEndEffectorParams", &GetEndEffectorParamsService);
    node->create_service<ros2_dobot::srv::SetEndEffectorLaser>("/DobotServer/SetEndEffectorLaser", &SetEndEffectorLaserService);
    node->create_service<ros2_dobot::srv::GetEndEffectorLaser>("/DobotServer/GetEndEffectorLaser", &GetEndEffectorLaserService);
    node->create_service<ros2_dobot::srv::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup", &SetEndEffectorSuctionCupService);
    node->create_service<ros2_dobot::srv::GetEndEffectorSuctionCup>("/DobotServer/GetEndEffectorSuctionCup", &GetEndEffectorSuctionCupService);
    node->create_service<ros2_dobot::srv::SetEndEffectorGripper>("/DobotServer/SetEndEffectorGripper", &SetEndEffectorGripperService);
    node->create_service<ros2_dobot::srv::GetEndEffectorGripper>("/DobotServer/GetEndEffectorGripper", &GetEndEffectorGripperService);

    // PTP Services
    node->create_service<ros2_dobot::srv::SetPTPJointParams>("/DobotServer/SetPTPJointParams", &SetPTPJointParamsService);
    node->create_service<ros2_dobot::srv::GetPTPJointParams>("/DobotServer/GetPTPJointParams", &GetPTPJointParamsService);
    node->create_service<ros2_dobot::srv::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams", &SetPTPCoordinateParamsService);
    node->create_service<ros2_dobot::srv::GetPTPCoordinateParams>("/DobotServer/GetPTPCoordinateParams", &GetPTPCoordinateParamsService);
    node->create_service<ros2_dobot::srv::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams", &SetPTPJumpParamsService);
    node->create_service<ros2_dobot::srv::GetPTPJumpParams>("/DobotServer/GetPTPJumpParams", GetPTPJumpParamsService);
    node->create_service<ros2_dobot::srv::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams", SetPTPCommonParamsService);
    node->create_service<ros2_dobot::srv::GetPTPCommonParams>("/DobotServer/GetPTPCommonParams", GetPTPCommonParamsService);
    node->create_service<ros2_dobot::srv::SetPTPCmd>("/DobotServer/SetPTPCmd", SetPTPCmdService);

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
    CreateServices(node);
    
    RCLCPP_INFO(node->get_logger(), "Dobot Server Node running...");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Dobot Server Node exiting...");
    
    // Disconnect Dobot
    DisconnectDobot();
    rclcpp::shutdown();
}

