#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <memory>
#include "DobotDll.h"



/*
 * Cmd timeout
 */
#include "dobot_interfaces/srv/set_cmd_timeout.hpp"


#include "dobot_interfaces/msg/pose_message.hpp"

using namespace std::chrono_literals;


void SetCmdTimeoutService( std::shared_ptr<dobot_interfaces::srv::SetCmdTimeout::Request> req, std::shared_ptr<dobot_interfaces::srv::SetCmdTimeout::Response> res)
{
    res->result = SetCmdTimeout(req->timeout);
}


/*
 * Device information
 */
#include "dobot_interfaces/srv/get_device_sn.hpp"
#include "dobot_interfaces/srv/set_device_name.hpp"
#include "dobot_interfaces/srv/get_device_name.hpp"
#include "dobot_interfaces/srv/get_device_version.hpp"

void GetDeviceSNService(std::shared_ptr<dobot_interfaces::srv::GetDeviceSN::Request> req, std::shared_ptr<dobot_interfaces::srv::GetDeviceSN::Response> res)
{
    char deviceSN[256];
    res->result = GetDeviceSN(deviceSN, sizeof(deviceSN));
    if (res->result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceSN;
        res->device_sn = ss.str();
    }
}

void SetDeviceNameService(std::shared_ptr<dobot_interfaces::srv::SetDeviceName::Request> req, std::shared_ptr<dobot_interfaces::srv::SetDeviceName::Response> res)
{
    res->result = SetDeviceName(req->device_name.c_str());
}

void GetDeviceNameService(std::shared_ptr<dobot_interfaces::srv::GetDeviceName::Request> req, std::shared_ptr<dobot_interfaces::srv::GetDeviceName::Response> res)
{
    char deviceName[256];

    res->result = GetDeviceName(deviceName, sizeof(deviceName));
    if (res->result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceName; 
        res->device_name = ss.str();
    }
}

void GetDeviceVersionService(std::shared_ptr<dobot_interfaces::srv::GetDeviceVersion::Request> req, std::shared_ptr<dobot_interfaces::srv::GetDeviceVersion::Response> res) 
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
#include "dobot_interfaces/srv/get_alarms_state.hpp"
#include "dobot_interfaces/srv/clear_all_alarms_state.hpp"

void GetAlarmsStateService(std::shared_ptr<dobot_interfaces::srv::GetAlarmsState::Request> req, std::shared_ptr<dobot_interfaces::srv::GetAlarmsState::Response> res)
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

void ClearAllAlarmsStateService(std::shared_ptr<dobot_interfaces::srv::ClearAllAlarmsState::Request> req, std::shared_ptr<dobot_interfaces::srv::ClearAllAlarmsState::Response> res)
{
    res->result = ClearAllAlarmsState();
}


/*
 * Pose
 */
#include "dobot_interfaces/srv/get_pose.hpp"

void GetPoseService(std::shared_ptr<dobot_interfaces::srv::GetPose::Request> req, std::shared_ptr<dobot_interfaces::srv::GetPose::Response> res)
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
#include "dobot_interfaces/srv/set_home_params.hpp"
#include "dobot_interfaces/srv/get_home_params.hpp"
#include "dobot_interfaces/srv/set_home_cmd.hpp"

void SetHOMEParamsService(std::shared_ptr<dobot_interfaces::srv::SetHOMEParams::Request> req, std::shared_ptr<dobot_interfaces::srv::SetHOMEParams::Response> res)
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

void GetHOMEParamsService(std::shared_ptr<dobot_interfaces::srv::GetHOMEParams::Request> req, std::shared_ptr<dobot_interfaces::srv::GetHOMEParams::Response> res)
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

void SetHOMECmdService(std::shared_ptr<dobot_interfaces::srv::SetHOMECmd::Request> req, std::shared_ptr<dobot_interfaces::srv::SetHOMECmd::Response> res)
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
#include "dobot_interfaces/srv/set_queued_cmd_start_exec.hpp"
#include "dobot_interfaces/srv/set_queued_cmd_stop_exec.hpp"
#include "dobot_interfaces/srv/set_queued_cmd_force_stop_exec.hpp"
#include "dobot_interfaces/srv/set_queued_cmd_clear.hpp"

void SetQueuedCmdStartExecService(std::shared_ptr<dobot_interfaces::srv::SetQueuedCmdStartExec::Request> req, std::shared_ptr<dobot_interfaces::srv::SetQueuedCmdStartExec::Response> res)
{
    res->result = SetQueuedCmdStartExec();
}

void SetQueuedCmdStopExecService(std::shared_ptr<dobot_interfaces::srv::SetQueuedCmdStopExec::Request> req, std::shared_ptr<dobot_interfaces::srv::SetQueuedCmdStopExec::Response> res)
{
    res->result = SetQueuedCmdStopExec();
}

void SetQueuedCmdForceStopExecService(std::shared_ptr<dobot_interfaces::srv::SetQueuedCmdForceStopExec::Request> req, std::shared_ptr<dobot_interfaces::srv::SetQueuedCmdForceStopExec::Response> res)
{
    res->result = SetQueuedCmdForceStopExec();
}

void SetQueuedCmdClearService(std::shared_ptr<dobot_interfaces::srv::SetQueuedCmdClear::Request> req, std::shared_ptr<dobot_interfaces::srv::SetQueuedCmdClear::Response> res)
{
    res->result = SetQueuedCmdClear();
}

/*
 * End effector
 */
#include "dobot_interfaces/srv/set_end_effector_params.hpp"
#include "dobot_interfaces/srv/get_end_effector_params.hpp"
#include "dobot_interfaces/srv/set_end_effector_laser.hpp"
#include "dobot_interfaces/srv/get_end_effector_laser.hpp"
#include "dobot_interfaces/srv/set_end_effector_suction_cup.hpp"
#include "dobot_interfaces/srv/get_end_effector_suction_cup.hpp"
#include "dobot_interfaces/srv/set_end_effector_gripper.hpp"
#include "dobot_interfaces/srv/get_end_effector_gripper.hpp"

void SetEndEffectorParamsService(std::shared_ptr<dobot_interfaces::srv::SetEndEffectorParams::Request> req, std::shared_ptr<dobot_interfaces::srv::SetEndEffectorParams::Response> res)
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
    
void GetEndEffectorParamsService(std::shared_ptr<dobot_interfaces::srv::GetEndEffectorParams::Request> req, std::shared_ptr<dobot_interfaces::srv::GetEndEffectorParams::Response> res)
{
    EndEffectorParams params;

    res->result = GetEndEffectorParams(&params);
    if (res->result == DobotCommunicate_NoError) {
        res->x_bias = params.xBias;
        res->y_bias = params.yBias;
        res->z_bias = params.zBias;
    }
}

void SetEndEffectorLaserService(std::shared_ptr<dobot_interfaces::srv::SetEndEffectorLaser::Request> req, std::shared_ptr<dobot_interfaces::srv::SetEndEffectorLaser::Response> res)
{
    uint64_t queued_cmd_index;

    res->result = SetEndEffectorLaser(req->enable_ctrl, req->on, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}

void GetEndEffectorLaserService(std::shared_ptr<dobot_interfaces::srv::GetEndEffectorLaser::Request> req, std::shared_ptr<dobot_interfaces::srv::GetEndEffectorLaser::Response> res)
{
    bool enable_ctrl, on;

    res->result = GetEndEffectorLaser(&enable_ctrl, &on);
    if (res->result == DobotCommunicate_NoError) {
        res->enable_ctrl = enable_ctrl;
        res->on = on;
    }

}

void SetEndEffectorSuctionCupService(std::shared_ptr<dobot_interfaces::srv::SetEndEffectorSuctionCup::Request> req, std::shared_ptr<dobot_interfaces::srv::SetEndEffectorSuctionCup::Response> res)
{
    uint64_t queued_cmd_index;

    res->result = SetEndEffectorSuctionCup(req->enable_ctrl, req->suck, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}
void GetEndEffectorSuctionCupService(std::shared_ptr<dobot_interfaces::srv::GetEndEffectorSuctionCup::Request> req, std::shared_ptr<dobot_interfaces::srv::GetEndEffectorSuctionCup::Response> res)
{
    bool enable_ctrl, suck;

    res->result = GetEndEffectorLaser(&enable_ctrl, &suck);
    if (res->result == DobotCommunicate_NoError) {
        res->enable_ctrl = enable_ctrl;
        res->suck = suck;
    }

}

void SetEndEffectorGripperService(std::shared_ptr<dobot_interfaces::srv::SetEndEffectorGripper::Request> req, std::shared_ptr<dobot_interfaces::srv::SetEndEffectorGripper::Response> res)
{
    uint64_t queued_cmd_index;

    res->result = SetEndEffectorGripper(req->enable_ctrl, req->grip, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}

void GetEndEffectorGripperService(std::shared_ptr<dobot_interfaces::srv::GetEndEffectorGripper::Request> req, std::shared_ptr<dobot_interfaces::srv::GetEndEffectorGripper::Response> res)
{
    bool enable_ctrl, grip;

    res->result = GetEndEffectorLaser(&enable_ctrl, &grip);
    if (res->result == DobotCommunicate_NoError) {
        res->enable_ctrl = enable_ctrl;
        res->grip = grip;
    }

}


/*
 * PTP
 */
#include "dobot_interfaces/srv/set_ptp_joint_params.hpp"
#include "dobot_interfaces/srv/get_ptp_joint_params.hpp"
#include "dobot_interfaces/srv/set_ptp_coordinate_params.hpp"
#include "dobot_interfaces/srv/get_ptp_coordinate_params.hpp"
#include "dobot_interfaces/srv/set_ptp_jump_params.hpp"
#include "dobot_interfaces/srv/get_ptp_jump_params.hpp"
#include "dobot_interfaces/srv/set_ptp_common_params.hpp"
#include "dobot_interfaces/srv/get_ptp_common_params.hpp"
#include "dobot_interfaces/srv/set_ptp_cmd.hpp"
    
void SetPTPJointParamsService(std::shared_ptr<dobot_interfaces::srv::SetPTPJointParams::Request> req, std::shared_ptr<dobot_interfaces::srv::SetPTPJointParams::Response> res)
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

void GetPTPJointParamsService(std::shared_ptr<dobot_interfaces::srv::GetPTPJointParams::Request> req, std::shared_ptr<dobot_interfaces::srv::GetPTPJointParams::Response> res)
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

void SetPTPCoordinateParamsService(std::shared_ptr<dobot_interfaces::srv::SetPTPCoordinateParams::Request> req, std::shared_ptr<dobot_interfaces::srv::SetPTPCoordinateParams::Response> res)
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

void GetPTPCoordinateParamsService(std::shared_ptr<dobot_interfaces::srv::GetPTPCoordinateParams::Request> req, std::shared_ptr<dobot_interfaces::srv::GetPTPCoordinateParams::Response> res)
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

void SetPTPJumpParamsService(std::shared_ptr<dobot_interfaces::srv::SetPTPJumpParams::Request> req, std::shared_ptr<dobot_interfaces::srv::SetPTPJumpParams::Response> res)
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

void GetPTPJumpParamsService(std::shared_ptr<dobot_interfaces::srv::GetPTPJumpParams::Request> req, std::shared_ptr<dobot_interfaces::srv::GetPTPJumpParams::Response> res)
{
    PTPJumpParams params;

    res->result = GetPTPJumpParams(&params);
    if (res->result == DobotCommunicate_NoError) {
        res->jump_height = params.jumpHeight;
        res->z_limit = params.zLimit;
    }
}

void SetPTPCommonParamsService(std::shared_ptr<dobot_interfaces::srv::SetPTPCommonParams::Request> req, std::shared_ptr<dobot_interfaces::srv::SetPTPCommonParams::Response> res)
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

void SetPTPCmdService(std::shared_ptr<dobot_interfaces::srv::SetPTPCmd::Request> req, std::shared_ptr<dobot_interfaces::srv::SetPTPCmd::Response> res)
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


void GetPTPCommonParamsService(std::shared_ptr<dobot_interfaces::srv::GetPTPCommonParams::Request> req, std::shared_ptr<dobot_interfaces::srv::GetPTPCommonParams::Response> res)
{
    PTPCommonParams params;

    res->result = GetPTPCommonParams(&params);
    if (res->result == DobotCommunicate_NoError) {
        res->velocity_ratio = params.velocityRatio;
        res->acceleration_ratio = params.accelerationRatio;
    }
}

/*
 * CP
 */
#include "dobot_interfaces/srv/set_cp_params.hpp"
#include "dobot_interfaces/srv/get_cp_params.hpp"
#include "dobot_interfaces/srv/set_cp_cmd.hpp"

void SetCPParamsService(std::shared_ptr<dobot_interfaces::srv::SetCPParams::Request> req, std::shared_ptr<dobot_interfaces::srv::SetCPParams::Response> res)
{
    CPParams params;
    uint64_t queued_cmd_index;

    params.planAcc = req->plan_acc;
    params.juncitionVel = req->junction_vel;
    params.acc = req->acc;
    params.realTimeTrack = req->real_time_track;
    res->result = SetCPParams(&params, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}

void GetCPParamsService(std::shared_ptr<dobot_interfaces::srv::GetCPParams::Request> req, std::shared_ptr<dobot_interfaces::srv::GetCPParams::Response> res)
{
    CPParams params;

    res->result = GetCPParams(&params);
    if (res->result == DobotCommunicate_NoError) {
        res->plan_acc = params.planAcc;
        res->junction_vel = params.juncitionVel;
        res->acc = params.acc;
        res->real_time_track = params.realTimeTrack;
    }

}

void SetCPCmdService(std::shared_ptr<dobot_interfaces::srv::SetCPCmd::Request> req, std::shared_ptr<dobot_interfaces::srv::SetCPCmd::Response> res)
{
    CPCmd cmd;
    uint64_t queued_cmd_index;

    cmd.cpMode = req->cp_mode;
    cmd.x = req->x;
    cmd.y = req->y;
    cmd.z = req->z;
    cmd.velocity = req->velocity;

    res->result = SetCPCmd(&cmd, true, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}

/*
 * ARC
 */
#include "dobot_interfaces/srv/set_arc_params.hpp"
#include "dobot_interfaces/srv/get_arc_params.hpp"
#include "dobot_interfaces/srv/set_arc_cmd.hpp"

void SetARCParamsService(std::shared_ptr<dobot_interfaces::srv::SetARCParams::Request> req, std::shared_ptr<dobot_interfaces::srv::SetARCParams::Response> res)
{
    ARCParams params;
    uint64_t queued_cmd_index;

    params.xyzVelocity = req->xyz_velocity;
    params.rVelocity = req->r_velocity;
    params.xyzAcceleration = req->xyz_acceleration;
    params.rAcceleration = req->r_acceleration;
    res->result = SetARCParams(&params, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}

void GetARCParamsService(std::shared_ptr<dobot_interfaces::srv::GetARCParams::Request> req, std::shared_ptr<dobot_interfaces::srv::GetARCParams::Response> res)
{
    ARCParams params;

    res->result = GetARCParams(&params);
    if (res->result == DobotCommunicate_NoError) {
        res->xyz_velocity = params.xyzVelocity;
        res->r_velocity = params.rVelocity;
        res->xyz_acceleration = params.xyzAcceleration;
        res->r_acceleration = params.rAcceleration;
    }

}

void SetARCCmdService(std::shared_ptr<dobot_interfaces::srv::SetARCCmd::Request> req, std::shared_ptr<dobot_interfaces::srv::SetARCCmd::Response> res)
{
    ARCCmd cmd;
    uint64_t queued_cmd_index;

    cmd.cirPoint.x = req->x1;
    cmd.cirPoint.y = req->y1;
    cmd.cirPoint.z = req->z1;
    cmd.cirPoint.r = req->r1;
    cmd.toPoint.x = req->x2;
    cmd.toPoint.y = req->y2;
    cmd.toPoint.z = req->z2;
    cmd.toPoint.r = req->r2;

    res->result = SetARCCmd(&cmd, true, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}

/*
 * WAIT
 */
#include "dobot_interfaces/srv/set_wait_cmd.hpp"

void SetWAITCmdService(std::shared_ptr<dobot_interfaces::srv::SetWAITCmd::Request> req, std::shared_ptr<dobot_interfaces::srv::SetWAITCmd::Response> res)
{
    WAITCmd cmd;
    uint64_t queued_cmd_index;

    cmd.timeout = req->timeout;
    res->result = SetWAITCmd(&cmd, true, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}

/*
 * TRIG
 */
#include "dobot_interfaces/srv/set_trig_cmd.hpp"

void SetTRIGCmdService(std::shared_ptr<dobot_interfaces::srv::SetTRIGCmd::Request> req, std::shared_ptr<dobot_interfaces::srv::SetTRIGCmd::Response> res)
{
    TRIGCmd cmd;
    uint64_t queued_cmd_index;

    cmd.address = req->address;
    cmd.mode = req->mode;
    cmd.condition = req->condition;
    cmd.threshold = req->threshold;
    res->result = SetTRIGCmd(&cmd, true, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}

/*
 * EIO
 */
#include "dobot_interfaces/srv/set_io_multiplexing.hpp"
#include "dobot_interfaces/srv/get_io_multiplexing.hpp"
#include "dobot_interfaces/srv/set_iodo.hpp"
#include "dobot_interfaces/srv/get_iodo.hpp"
#include "dobot_interfaces/srv/set_iopwm.hpp"
#include "dobot_interfaces/srv/get_iopwm.hpp"
#include "dobot_interfaces/srv/get_iodi.hpp"
#include "dobot_interfaces/srv/get_ioadc.hpp"
#include "dobot_interfaces/srv/set_e_motor.hpp"
#include "dobot_interfaces/srv/set_infrared_sensor.hpp"
#include "dobot_interfaces/srv/get_infrared_sensor.hpp"
#include "dobot_interfaces/srv/set_color_sensor.hpp"
#include "dobot_interfaces/srv/get_color_sensor.hpp"

void SetIOMultiplexingService(std::shared_ptr<dobot_interfaces::srv::SetIOMultiplexing::Request> req, std::shared_ptr<dobot_interfaces::srv::SetIOMultiplexing::Response> res)
{
    IOMultiplexing ioMultiplexing;
    uint64_t queued_cmd_index;

    ioMultiplexing.address = req->address;
    ioMultiplexing.multiplex = req->multiplex;
    res->result = SetIOMultiplexing(&ioMultiplexing, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}
void GetIOMultiplexingService(std::shared_ptr<dobot_interfaces::srv::GetIOMultiplexing::Request> req, std::shared_ptr<dobot_interfaces::srv::GetIOMultiplexing::Response> res)
{
    IOMultiplexing ioMultiplexing;

    ioMultiplexing.address = req->address;
    res->result = GetIOMultiplexing(&ioMultiplexing);
    if (res->result == DobotCommunicate_NoError) {
        res->multiplex = ioMultiplexing.multiplex;
    }

}

void SetIODOService(std::shared_ptr<dobot_interfaces::srv::SetIODO::Request> req, std::shared_ptr<dobot_interfaces::srv::SetIODO::Response> res)
{
    IODO ioDO;
    uint64_t queued_cmd_index;

    ioDO.address = req->address;
    ioDO.level = req->level;
    res->result = SetIODO(&ioDO, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}

void GetIODOService(std::shared_ptr<dobot_interfaces::srv::GetIODO::Request> req, std::shared_ptr<dobot_interfaces::srv::GetIODO::Response> res)
{
    IODO ioDO;

    ioDO.address = req->address;
    res->result = GetIODO(&ioDO);
    if (res->result == DobotCommunicate_NoError) {
        res->level = ioDO.level;
    }

}

void SetIOPWMService(std::shared_ptr<dobot_interfaces::srv::SetIOPWM::Request> req, std::shared_ptr<dobot_interfaces::srv::SetIOPWM::Response> res)
{
    IOPWM ioPWM;
    uint64_t queued_cmd_index;

    ioPWM.address = req->address;
    ioPWM.frequency = req->frequency;
    ioPWM.dutyCycle = req->duty_cycle;
    res->result = SetIOPWM(&ioPWM, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}

void GetIOPWMService(std::shared_ptr<dobot_interfaces::srv::GetIOPWM::Request> req, std::shared_ptr<dobot_interfaces::srv::GetIOPWM::Response> res)
{
    IOPWM ioPWM;

    ioPWM.address = req->address;
    res->result = GetIOPWM(&ioPWM);
    if (res->result == DobotCommunicate_NoError) {
        res->frequency = ioPWM.frequency;
        res->duty_cycle = ioPWM.dutyCycle;
    }

}

void GetIODIService(std::shared_ptr<dobot_interfaces::srv::GetIODI::Request> req, std::shared_ptr<dobot_interfaces::srv::GetIODI::Response> res)
{
    IODI ioDI;

    ioDI.address = req->address;
    res->result = GetIODI(&ioDI);
    if (res->result == DobotCommunicate_NoError) {
        res->level = ioDI.level;
    }

}

void GetIOADCService(std::shared_ptr<dobot_interfaces::srv::GetIOADC::Request> req, std::shared_ptr<dobot_interfaces::srv::GetIOADC::Response> res)
{
    IOADC ioADC;

    ioADC.address = req->address;
    res->result = GetIOADC(&ioADC);
    if (res->result == DobotCommunicate_NoError) {
        res->value = ioADC.value;
    }

}

void SetEMotorService(std::shared_ptr<dobot_interfaces::srv::SetEMotor::Request> req, std::shared_ptr<dobot_interfaces::srv::SetEMotor::Response> res)
{
    EMotor eMotor;
    uint64_t queued_cmd_index;

    eMotor.index = req->index;
    eMotor.isEnabled = req->is_enabled;
    eMotor.speed = req->speed;
    res->result = SetEMotor(&eMotor, req->is_queued, &queued_cmd_index);
    if (res->result == DobotCommunicate_NoError) {
        res->queued_cmd_index = queued_cmd_index;
    }

}


void SetInfraredSensorService(std::shared_ptr<dobot_interfaces::srv::SetInfraredSensor::Request> req, std::shared_ptr<dobot_interfaces::srv::SetInfraredSensor::Response> res)
{
    InfraredPort infrared_port = InfraredPort(req->infrared_port);
    res->result = SetInfraredSensor(req->enable_ctrl, infrared_port, 0);

}

void GetInfraredSensorService(std::shared_ptr<dobot_interfaces::srv::GetInfraredSensor::Request> req, std::shared_ptr<dobot_interfaces::srv::GetInfraredSensor::Response> res)
{
    uint8_t value;
    InfraredPort infrared_port = InfraredPort(req->infrared_port);
    res->result = GetInfraredSensor(infrared_port, &value);
    if (res->result == DobotCommunicate_NoError) {
        res->value = value;
    }

}

void SetColorSensorService(std::shared_ptr<dobot_interfaces::srv::SetColorSensor::Request> req, std::shared_ptr<dobot_interfaces::srv::SetColorSensor::Response> res)
{
    ColorPort color_port = ColorPort(req->color_port);
    res->result = SetColorSensor(req->enable_ctrl, color_port, 0);

}

void GetColorSensorService(std::shared_ptr<dobot_interfaces::srv::GetColorSensor::Request> req, std::shared_ptr<dobot_interfaces::srv::GetColorSensor::Response> res)
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    res->result = GetColorSensor(&r, &g, &b);
    if (res->result == DobotCommunicate_NoError) {
        res->r = r;
        res->g = g;
        res->b = b;
    }

}

struct ServiceInstances
{
rclcpp::Service<dobot_interfaces::srv::ClearAllAlarmsState>::SharedPtr ClearAllAlarmsState;
rclcpp::Service<dobot_interfaces::srv::GetAlarmsState>::SharedPtr GetAlarmsState;
rclcpp::Service<dobot_interfaces::srv::GetARCParams>::SharedPtr GetARCParams;
rclcpp::Service<dobot_interfaces::srv::GetColorSensor>::SharedPtr GetColorSensor;
rclcpp::Service<dobot_interfaces::srv::GetCPParams>::SharedPtr GetCPParams;
rclcpp::Service<dobot_interfaces::srv::GetDeviceName>::SharedPtr GetDeviceName;
rclcpp::Service<dobot_interfaces::srv::GetDeviceSN>::SharedPtr GetDeviceSN;
rclcpp::Service<dobot_interfaces::srv::GetDeviceVersion>::SharedPtr GetDeviceVersion;
rclcpp::Service<dobot_interfaces::srv::GetEndEffectorGripper>::SharedPtr GetEndEffectorGripper;
rclcpp::Service<dobot_interfaces::srv::GetEndEffectorLaser>::SharedPtr GetEndEffectorLaser;
rclcpp::Service<dobot_interfaces::srv::GetEndEffectorParams>::SharedPtr GetEndEffectorParams;
rclcpp::Service<dobot_interfaces::srv::GetEndEffectorSuctionCup>::SharedPtr GetEndEffectorSuctionCup;
rclcpp::Service<dobot_interfaces::srv::GetHOMEParams>::SharedPtr GetHOMEParams;
rclcpp::Service<dobot_interfaces::srv::GetInfraredSensor>::SharedPtr GetInfraredSensor;
rclcpp::Service<dobot_interfaces::srv::GetIOADC>::SharedPtr GetIOADC;
rclcpp::Service<dobot_interfaces::srv::GetIODI>::SharedPtr GetIODI;
rclcpp::Service<dobot_interfaces::srv::GetIODO>::SharedPtr GetIODO;
rclcpp::Service<dobot_interfaces::srv::GetIOMultiplexing>::SharedPtr GetIOMultiplexing;
rclcpp::Service<dobot_interfaces::srv::GetIOPWM>::SharedPtr GetIOPWM;
rclcpp::Service<dobot_interfaces::srv::GetPose>::SharedPtr GetPose;
rclcpp::Service<dobot_interfaces::srv::GetPTPCommonParams>::SharedPtr GetPTPCommonParams;
rclcpp::Service<dobot_interfaces::srv::GetPTPCoordinateParams>::SharedPtr GetPTPCoordinateParams;
rclcpp::Service<dobot_interfaces::srv::GetPTPJointParams>::SharedPtr GetPTPJointParams;
rclcpp::Service<dobot_interfaces::srv::GetPTPJumpParams>::SharedPtr GetPTPJumpParams;
rclcpp::Service<dobot_interfaces::srv::SetARCCmd>::SharedPtr SetARCCmd;
rclcpp::Service<dobot_interfaces::srv::SetARCParams>::SharedPtr SetARCParams;
rclcpp::Service<dobot_interfaces::srv::SetCmdTimeout>::SharedPtr SetCmdTimeout;
rclcpp::Service<dobot_interfaces::srv::SetColorSensor>::SharedPtr SetColorSensor;
rclcpp::Service<dobot_interfaces::srv::SetCPCmd>::SharedPtr SetCPCmd;
rclcpp::Service<dobot_interfaces::srv::SetCPParams>::SharedPtr SetCPParams;
rclcpp::Service<dobot_interfaces::srv::SetDeviceName>::SharedPtr SetDeviceName;
rclcpp::Service<dobot_interfaces::srv::SetEMotor>::SharedPtr SetEMotor;
rclcpp::Service<dobot_interfaces::srv::SetEndEffectorGripper>::SharedPtr SetEndEffectorGripper;
rclcpp::Service<dobot_interfaces::srv::SetEndEffectorLaser>::SharedPtr SetEndEffectorLaser;
rclcpp::Service<dobot_interfaces::srv::SetEndEffectorParams>::SharedPtr SetEndEffectorParams;
rclcpp::Service<dobot_interfaces::srv::SetEndEffectorSuctionCup>::SharedPtr SetEndEffectorSuctionCup;
rclcpp::Service<dobot_interfaces::srv::SetHOMECmd>::SharedPtr SetHOMECmd;
rclcpp::Service<dobot_interfaces::srv::SetHOMEParams>::SharedPtr SetHOMEParams;
rclcpp::Service<dobot_interfaces::srv::SetInfraredSensor>::SharedPtr SetInfraredSensor;
rclcpp::Service<dobot_interfaces::srv::SetIODO>::SharedPtr SetIODO;
rclcpp::Service<dobot_interfaces::srv::SetIOMultiplexing>::SharedPtr SetIOMultiplexing;
rclcpp::Service<dobot_interfaces::srv::SetIOPWM>::SharedPtr SetIOPWM;
rclcpp::Service<dobot_interfaces::srv::SetPTPCmd>::SharedPtr SetPTPCmd;
rclcpp::Service<dobot_interfaces::srv::SetPTPCommonParams>::SharedPtr SetPTPCommonParams;
rclcpp::Service<dobot_interfaces::srv::SetPTPCoordinateParams>::SharedPtr SetPTPCoordinateParams;
rclcpp::Service<dobot_interfaces::srv::SetPTPJointParams>::SharedPtr SetPTPJointParams;
rclcpp::Service<dobot_interfaces::srv::SetPTPJumpParams>::SharedPtr SetPTPJumpParams;
rclcpp::Service<dobot_interfaces::srv::SetQueuedCmdClear>::SharedPtr SetQueuedCmdClear;
rclcpp::Service<dobot_interfaces::srv::SetQueuedCmdForceStopExec>::SharedPtr SetQueuedCmdForceStopExec;
rclcpp::Service<dobot_interfaces::srv::SetQueuedCmdStartExec>::SharedPtr SetQueuedCmdStartExec;
rclcpp::Service<dobot_interfaces::srv::SetQueuedCmdStopExec>::SharedPtr SetQueuedCmdStopExec;
rclcpp::Service<dobot_interfaces::srv::SetTRIGCmd>::SharedPtr SetTRIGCmd;
rclcpp::Service<dobot_interfaces::srv::SetWAITCmd>::SharedPtr SetWAITCmd;

  
};

/**
 * Create services
 */
 void CreateServices(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ServiceInstances> serviceInstances, std::string nameSpace, rclcpp::CallbackGroup::SharedPtr serviceCBGroup) {
    // ServiceInstances serviceInstances;
    serviceInstances->SetCmdTimeout = node->create_service<dobot_interfaces::srv::SetCmdTimeout>("SetCmdTimeout", &SetCmdTimeoutService, rmw_qos_profile_services_default, serviceCBGroup);

    // Device Information Services
    serviceInstances->GetDeviceSN = node->create_service<dobot_interfaces::srv::GetDeviceSN>("GetDeviceSN", &GetDeviceSNService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetDeviceName = node->create_service<dobot_interfaces::srv::SetDeviceName>("SetDeviceName", &SetDeviceNameService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetDeviceName  = node->create_service<dobot_interfaces::srv::GetDeviceName>("GetDeviceName", &GetDeviceNameService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetDeviceVersion  = node->create_service<dobot_interfaces::srv::GetDeviceVersion>("GetDeviceVersion", &GetDeviceVersionService, rmw_qos_profile_services_default, serviceCBGroup);


    // Alarm Services
    serviceInstances->GetAlarmsState  = node->create_service<dobot_interfaces::srv::GetAlarmsState>("GetAlarmsState", &GetAlarmsStateService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->ClearAllAlarmsState  = node->create_service<dobot_interfaces::srv::ClearAllAlarmsState>("ClearAllAlarmsState", &ClearAllAlarmsStateService, rmw_qos_profile_services_default, serviceCBGroup);

    // Pose Services
    serviceInstances->GetPose  = node->create_service<dobot_interfaces::srv::GetPose>("GetPoseService", &GetPoseService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetHOMEParams  = node->create_service<dobot_interfaces::srv::SetHOMEParams>("SetHOMEParams", &SetHOMEParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetHOMEParams  = node->create_service<dobot_interfaces::srv::GetHOMEParams>("GetHOMEParams", &GetHOMEParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetHOMECmd  = node->create_service<dobot_interfaces::srv::SetHOMECmd>("SetHOMECmd", &SetHOMECmdService, rmw_qos_profile_services_default, serviceCBGroup);

    // Queue Command Services
    serviceInstances->SetQueuedCmdClear  = node->create_service<dobot_interfaces::srv::SetQueuedCmdClear>("SetQueuedCmdClear", &SetQueuedCmdClearService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetQueuedCmdStartExec  = node->create_service<dobot_interfaces::srv::SetQueuedCmdStartExec>("SetQueuedCmdStartExec", &SetQueuedCmdStartExecService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetQueuedCmdStopExec  = node->create_service<dobot_interfaces::srv::SetQueuedCmdStopExec>("SetQueuedCmdStopExecService", &SetQueuedCmdStopExecService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetQueuedCmdForceStopExec  = node->create_service<dobot_interfaces::srv::SetQueuedCmdForceStopExec>("SetQueuedCmdForceStopExecService", &SetQueuedCmdForceStopExecService, rmw_qos_profile_services_default, serviceCBGroup);

    // End Effector Services
    serviceInstances->SetEndEffectorParams  = node->create_service<dobot_interfaces::srv::SetEndEffectorParams>("SetEndEffectorParams", &SetEndEffectorParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetEndEffectorParams  = node->create_service<dobot_interfaces::srv::GetEndEffectorParams>("GetEndEffectorParams", &GetEndEffectorParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetEndEffectorLaser  = node->create_service<dobot_interfaces::srv::SetEndEffectorLaser>("SetEndEffectorLaser", &SetEndEffectorLaserService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetEndEffectorLaser  = node->create_service<dobot_interfaces::srv::GetEndEffectorLaser>("GetEndEffectorLaser", &GetEndEffectorLaserService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetEndEffectorSuctionCup  = node->create_service<dobot_interfaces::srv::SetEndEffectorSuctionCup>("SetEndEffectorSuctionCup", &SetEndEffectorSuctionCupService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetEndEffectorSuctionCup  = node->create_service<dobot_interfaces::srv::GetEndEffectorSuctionCup>("GetEndEffectorSuctionCup", &GetEndEffectorSuctionCupService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetEndEffectorGripper  = node->create_service<dobot_interfaces::srv::SetEndEffectorGripper>("SetEndEffectorGripper", &SetEndEffectorGripperService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetEndEffectorGripper  = node->create_service<dobot_interfaces::srv::GetEndEffectorGripper>("GetEndEffectorGripper", &GetEndEffectorGripperService, rmw_qos_profile_services_default, serviceCBGroup);

    // PTP Services
    serviceInstances->SetPTPJointParams  = node->create_service<dobot_interfaces::srv::SetPTPJointParams>("SetPTPJointParams", &SetPTPJointParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetPTPJointParams  = node->create_service<dobot_interfaces::srv::GetPTPJointParams>("GetPTPJointParams", &GetPTPJointParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetPTPCoordinateParams  = node->create_service<dobot_interfaces::srv::SetPTPCoordinateParams>("SetPTPCoordinateParams", &SetPTPCoordinateParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetPTPCoordinateParams  = node->create_service<dobot_interfaces::srv::GetPTPCoordinateParams>("GetPTPCoordinateParams", &GetPTPCoordinateParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetPTPJumpParams  = node->create_service<dobot_interfaces::srv::SetPTPJumpParams>("SetPTPJumpParams", &SetPTPJumpParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetPTPJumpParams  = node->create_service<dobot_interfaces::srv::GetPTPJumpParams>("GetPTPJumpParams", &GetPTPJumpParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetPTPCommonParams  = node->create_service<dobot_interfaces::srv::SetPTPCommonParams>("SetPTPCommonParams", &SetPTPCommonParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetPTPCommonParams  = node->create_service<dobot_interfaces::srv::GetPTPCommonParams>("GetPTPCommonParams", &GetPTPCommonParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetPTPCmd  = node->create_service<dobot_interfaces::srv::SetPTPCmd>("SetPTPCmd", &SetPTPCmdService, rmw_qos_profile_services_default, serviceCBGroup);

    // CP Services
    serviceInstances->SetCPParams  = node->create_service<dobot_interfaces::srv::SetCPParams>("SetCPParams", &SetCPParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetCPParams  = node->create_service<dobot_interfaces::srv::GetCPParams>("GetCPParams", &GetCPParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetCPCmd  = node->create_service<dobot_interfaces::srv::SetCPCmd>("SetCPCmd", &SetCPCmdService, rmw_qos_profile_services_default, serviceCBGroup);

    // ARC Service
    serviceInstances->SetARCParams  = node->create_service<dobot_interfaces::srv::SetARCParams>("SetARCParams", &SetARCParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetARCParams  = node->create_service<dobot_interfaces::srv::GetARCParams>("GetARCParams", &GetARCParamsService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetARCCmd  = node->create_service<dobot_interfaces::srv::SetARCCmd>("SetARCCmd", &SetARCCmdService, rmw_qos_profile_services_default, serviceCBGroup);

    // WAIT services
        serviceInstances->SetWAITCmd  = node->create_service<dobot_interfaces::srv::SetWAITCmd>("SetWAITCmd", &SetWAITCmdService, rmw_qos_profile_services_default, serviceCBGroup);

    // TRIG Services
        serviceInstances->SetTRIGCmd  = node->create_service<dobot_interfaces::srv::SetTRIGCmd>("SetTRIGCmd", &SetTRIGCmdService, rmw_qos_profile_services_default, serviceCBGroup);

    // EIO Services
    serviceInstances->SetIOMultiplexing  = node->create_service<dobot_interfaces::srv::SetIOMultiplexing>("SetIOMultiplexing", &SetIOMultiplexingService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetIOMultiplexing  = node->create_service<dobot_interfaces::srv::GetIOMultiplexing>("GetIOMultiplexing", &GetIOMultiplexingService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetIODO  = node->create_service<dobot_interfaces::srv::SetIODO>("SetIODO", &SetIODOService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetIODO  = node->create_service<dobot_interfaces::srv::GetIODO>("GetIODO", &GetIODOService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetIOPWM  = node->create_service<dobot_interfaces::srv::SetIOPWM>("SetIOPWM", &SetIOPWMService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetIOPWM  = node->create_service<dobot_interfaces::srv::GetIOPWM>("GetIOPWM", &GetIOPWMService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetIODI  = node->create_service<dobot_interfaces::srv::GetIODI>("GetIODI", &GetIODIService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetIOADC = node->create_service<dobot_interfaces::srv::GetIOADC>("GetIOADC", &GetIOADCService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetEMotor  = node->create_service<dobot_interfaces::srv::SetEMotor>("SetEMotor", &SetEMotorService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetInfraredSensor  = node->create_service<dobot_interfaces::srv::SetInfraredSensor>("SetInfraredSensor", &SetInfraredSensorService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetInfraredSensor  = node->create_service<dobot_interfaces::srv::GetInfraredSensor>("GetInfraredSensor", &GetInfraredSensorService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->SetColorSensor  = node->create_service<dobot_interfaces::srv::SetColorSensor>("SetColorSensor", &SetColorSensorService, rmw_qos_profile_services_default, serviceCBGroup);
    serviceInstances->GetColorSensor = node->create_service<dobot_interfaces::srv::GetColorSensor>("GetColorSensor", &GetColorSensorService, rmw_qos_profile_services_default, serviceCBGroup);

}

rclcpp::Publisher<dobot_interfaces::msg::PoseMessage>::SharedPtr posePublisher;
void publishPose() {
    Pose pose;
    auto poseMessage = dobot_interfaces::msg::PoseMessage();
    auto result = GetPose(&pose);
    if (result == DobotCommunicate_NoError) {
        poseMessage.x = pose.x;
        poseMessage.y = pose.y;
        poseMessage.z = pose.z;
        poseMessage.r = pose.r;
        for (int i = 0; i < 4; i++) {
            poseMessage.joint_angle.push_back(pose.jointAngle[i]); 
        }
    }
    posePublisher->publish(poseMessage);
}


/**
 * Main
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cout << "Usage:  <portName> <node-name>" << std::endl;
        return -1;
    }
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(argv[2], argv[2]);
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
    std::shared_ptr<ServiceInstances> services = std::make_shared<ServiceInstances>();
    std::string nameSpace = argv[2];
    auto serviceCBGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    CreateServices(node, services, nameSpace, serviceCBGroup);

    // Setup the Pose publisher

    posePublisher = node->create_publisher<dobot_interfaces::msg::PoseMessage>("pose", 10 );
    auto timerCBGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto poseTimer = node->create_wall_timer(10s, &publishPose, timerCBGroup);

    RCLCPP_INFO(node->get_logger(), "Dobot Server Node is running.");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Dobot Server Node exiting");
    
    // Disconnect Dobot
    DisconnectDobot();
    rclcpp::shutdown();
}

