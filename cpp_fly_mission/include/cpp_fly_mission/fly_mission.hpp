#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/info/info.h>

#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <thread>
#include <string>
#include <utility>
#include <cmath>

#include <stdint.h>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::placeholders;
using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

namespace mission
{
    class FlyMission : public rclcpp::Node
    {
    public:
        FlyMission();
    private:
        std::unique_ptr<mavsdk::Mavsdk> _mavsdk;
        std::unique_ptr<mavsdk::Action> _action;
        std::shared_ptr<mavsdk::Telemetry> _telemetry;
        std::shared_ptr<mavsdk::Mission> _mission;
        std::unique_ptr<mavsdk::Offboard> _offboard;
        std::vector<mavsdk::Mission::MissionItem> mission_items;
        std::atomic<int> waypoint;
        std::vector<double> p1;
        std::vector<double> p2;
        std::vector<double> p_d;
        std::atomic<float> depthValue;
        //Offboard offboard;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _velocitySub;
        void cbVelocity(const geometry_msgs::msg::Twist::SharedPtr aMsg);

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvUpload;
        void cbUpload(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse);

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvTakeOff;
        void cbTakeOff(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse);

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStartMission;
        void cbStartMission(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse);

        void upload();
        void takeOff();
        std::shared_ptr<mavsdk::System> getSystem(mavsdk::Mavsdk& aMavsdk);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _depthSub;
        void cbDepth(const sensor_msgs::msg::Image::SharedPtr msg);
    };
}