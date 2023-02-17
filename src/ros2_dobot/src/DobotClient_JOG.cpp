#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "dobot_interfaces/srv/set_cmd_timeout.hpp"
#include "dobot_interfaces/srv/set_jog_cmd.hpp"
#include <cstdlib>

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_U 0x75
#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b

int kfd = 0;
struct termios cooked, raw;

void keyboardLoop(std::shared_ptr<rclcpp::Node> node)
{
    unsigned char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");

    auto logger = rclcpp::get_logger("");
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    auto setJOGCmdRequest = std::make_shared<dobot_interfaces::srv::SetJOGCmd::Request>();   
    auto setJOGCmdClient = node->create_client<dobot_interfaces::srv::SetJOGCmd>("/DobotServer/SetJOGCmd");
    setJOGCmdRequest->is_joint = false;
    for (;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if (num > 0)
        {
            if (read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            continue;
        }
        switch (c)
        {
        case KEYCODE_W:
            RCLCPP_INFO(logger, "W");
            setJOGCmdRequest->cmd = 1;
            break;
        case KEYCODE_S:
            RCLCPP_INFO(logger, "S");
            setJOGCmdRequest->cmd = 2;
            break;
        case KEYCODE_A:
            RCLCPP_INFO(logger, "A");
            setJOGCmdRequest->cmd = 3;
            break;
        case KEYCODE_D:
            RCLCPP_INFO(logger, "D");
            setJOGCmdRequest->cmd = 4;
            break;
        case KEYCODE_U:
            RCLCPP_INFO(logger, "U");
            setJOGCmdRequest->cmd = 5;
            break;
        case KEYCODE_I:
            RCLCPP_INFO(logger, "I");
            setJOGCmdRequest->cmd = 6;
            break;
        case KEYCODE_J:
            RCLCPP_INFO(logger, "J");
            setJOGCmdRequest->cmd = 7;
            break;
        case KEYCODE_K:
            RCLCPP_INFO(logger, "K");
            setJOGCmdRequest->cmd = 8;
            break;
        default:
            RCLCPP_INFO(logger, "DEFAULT:0x%02x", c);
            setJOGCmdRequest->cmd = 0;
            break;
        }
        auto result = setJOGCmdClient->async_send_request(setJOGCmdRequest);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            RCLCPP_INFO(logger, "Result:%d", response->result);
        }
    else
        {
            RCLCPP_ERROR(logger, "Failed to call SetJOGCMD");
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("DobotClient_JOG");
    auto logger = rclcpp::get_logger("DobotClient_JOG");
    RCLCPP_INFO(logger, "Connecting to DobotServer node ...");
    auto setCmdTimeoutclient = node->create_client<dobot_interfaces::srv::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
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

    boost::thread t = boost::thread(boost::bind(&keyboardLoop, node));
    rclcpp::spin(node);
    t.interrupt();
    t.join();

    tcsetattr(kfd, TCSANOW, &cooked);
    // return 0;
}
