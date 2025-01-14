#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "dobot_interfaces/srv/set_cmd_timeout.hpp"
#include "dobot_interfaces/srv/set_cmd_timeout.hpp"
#include "dobot_interfaces/srv/set_queued_cmd_clear.hpp"
#include "dobot_interfaces/srv/set_queued_cmd_start_exec.hpp"
#include "dobot_interfaces/srv/set_queued_cmd_force_stop_exec.hpp"
#include "dobot_interfaces/srv/get_device_version.hpp"

#include "dobot_interfaces/srv/set_end_effector_params.hpp"
#include "dobot_interfaces/srv/set_ptp_joint_params.hpp"
#include "dobot_interfaces/srv/set_ptp_coordinate_params.hpp"
#include "dobot_interfaces/srv/set_ptp_jump_params.hpp"
#include "dobot_interfaces/srv/set_ptp_common_params.hpp"
#include "dobot_interfaces/srv/set_ptp_cmd.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("DobotClient_PTP");
    auto logger = rclcpp::get_logger("DobotClient_PTP");
    auto setCmdTimeoutclient = node->create_client<dobot_interfaces::srv::SetCmdTimeout>("/DobotServer/SetCmdTimeout");

    RCLCPP_INFO(logger, "Connecting to DobotServer node ...");
    while (!setCmdTimeoutclient->wait_for_service(std::chrono::seconds(10)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(logger, "Interrupted while waiting for service");
            return 0;
        }
        RCLCPP_INFO(logger, "Service not available, waiting ...");
    }

    auto setCmdTimeoutRequest = std::make_shared<dobot_interfaces::srv::SetCmdTimeout::Request>();
    setCmdTimeoutRequest->timeout = 3000;
    setCmdTimeoutclient->async_send_request(setCmdTimeoutRequest);

    // Clear the command queue
    auto setQueuedCmdClearRequest = std::make_shared<dobot_interfaces::srv::SetQueuedCmdClear::Request>();
    node->create_client<dobot_interfaces::srv::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear")->async_send_request(setQueuedCmdClearRequest);

    // Start running the command queue
    auto setQueuedCmdStartExecRequest = std::make_shared<dobot_interfaces::srv::SetQueuedCmdStartExec::Request>();
    node->create_client<dobot_interfaces::srv::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec")->async_send_request(setQueuedCmdStartExecRequest);

    // Get device version information
    auto getDeviceVersionRequest = std::make_shared<dobot_interfaces::srv::GetDeviceVersion::Request>();
    auto getDeviceVersionClient = node->create_client<dobot_interfaces::srv::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    auto result = getDeviceVersionClient->async_send_request(getDeviceVersionRequest);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        RCLCPP_INFO(logger, "Device version:%d.%d.%d", response->major_version, response->minor_version, response->revision);
    }
    else
    {
        RCLCPP_ERROR(logger, "Failed to call server GetDeviceVersion");
    }

    // Set end effector parameters
    auto setEndEffectorParamsRequest = std::make_shared<dobot_interfaces::srv::SetEndEffectorParams::Request>();
    setEndEffectorParamsRequest->x_bias = 70;
    setEndEffectorParamsRequest->y_bias = 0;
    setEndEffectorParamsRequest->z_bias = 0;
    node->create_client<dobot_interfaces::srv::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams")->async_send_request(setEndEffectorParamsRequest);

    // Set PTP joint parameters
    auto setPTPJointParamsRequest = std::make_shared<dobot_interfaces::srv::SetPTPJointParams::Request>();
    for (int i = 0; i < 4; i++)
    {
        setPTPJointParamsRequest->velocity.push_back(100);
    }
    for (int i = 0; i < 4; i++)
    {
        setPTPJointParamsRequest->acceleration.push_back(100);
    }
    node->create_client<dobot_interfaces::srv::SetPTPJointParams>("/DobotServer/SetPTPJointParams")->async_send_request(setPTPJointParamsRequest);

    // Set PTP coordinate parameters
    auto setPTPCoordinateParamsRequest = std::make_shared<dobot_interfaces::srv::SetPTPCoordinateParams::Request>();
    setPTPCoordinateParamsRequest->xyz_velocity = 100;
    setPTPCoordinateParamsRequest->xyz_acceleration = 100;
    setPTPCoordinateParamsRequest->r_velocity = 100;
    setPTPCoordinateParamsRequest->r_acceleration = 100;
    node->create_client<dobot_interfaces::srv::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams")->async_send_request(setPTPCoordinateParamsRequest);

    // Set PTP jump parameters
    auto setPTPJumpParamsRequest = std::make_shared<dobot_interfaces::srv::SetPTPJumpParams::Request>();
    setPTPJumpParamsRequest->jump_height = 20;
    setPTPJumpParamsRequest->z_limit = 200;
    node->create_client<dobot_interfaces::srv::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams")->async_send_request(setPTPJumpParamsRequest);

    // Set PTP common parameters
    auto setPTPCommonParamsRequest = std::make_shared<dobot_interfaces::srv::SetPTPCommonParams::Request>();
    setPTPCommonParamsRequest->velocity_ratio = 50;
    setPTPCommonParamsRequest->acceleration_ratio = 50;
    node->create_client<dobot_interfaces::srv::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams")->async_send_request(setPTPCommonParamsRequest);

    auto setPTPCmdRequest = std::make_shared<dobot_interfaces::srv::SetPTPCmd::Request>();
    auto setPTPCmdClient = node->create_client<dobot_interfaces::srv::SetPTPCmd>("/DobotServer/SetPTPCmd");

    while (rclcpp::ok())
    {
        RCLCPP_INFO(logger, "Moving the arm ..");
        // The first point
        do
        {
            RCLCPP_INFO(logger, "Moving to First point ..");
            setPTPCmdRequest->ptp_mode = 1;
            setPTPCmdRequest->x = 200;
            setPTPCmdRequest->y = 0;
            setPTPCmdRequest->z = 0;
            setPTPCmdRequest->r = 0;
            auto result = setPTPCmdClient->async_send_request(setPTPCmdRequest);
            if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                if (result.get()->result == 0)
                {
                    break;
                }
            }
        } while (1);

        // The second point
        do
        {
            RCLCPP_INFO(logger, "Moving to Second point ..");

            setPTPCmdRequest->ptp_mode = 1;
            setPTPCmdRequest->x = 250;
            setPTPCmdRequest->y = 0;
            setPTPCmdRequest->z = 0;
            setPTPCmdRequest->r = 0;
            auto result = setPTPCmdClient->async_send_request(setPTPCmdRequest);
            if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                if (result.get()->result == 0)
                {
                    break;
                }
            }
        } while (1);
        RCLCPP_INFO(logger, "Waiting for requests to complete ..");
        rclcpp::spin_some(node);
        rclcpp::shutdown();
        RCLCPP_INFO(logger, "Done!");
    }
    return 0;
}
