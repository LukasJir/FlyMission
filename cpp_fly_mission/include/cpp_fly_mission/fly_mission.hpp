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
        std::atomic<float> next_waypoint_latitude;
        std::atomic<float> next_waypoint_longitude;
        std::atomic<float> last_waypoint_latitude;
        std::atomic<float> last_waypoint_longitude;

        mavsdk::Telemetry::Position drone_start_pos;
        mavsdk::Telemetry::Position drone_pos;
        std::atomic<float> drone_latitude;
        std::atomic<float> drone_longitude;

        std::vector<float> p1;
        std::vector<float> p2;
        std::vector<float> p_d;

        std::atomic<float> x_d;
        std::atomic<float> y_d;
        std::atomic<float> distance_to_line;

        std::atomic<float> drone_x_norm;
        std::atomic<float> drone_y_norm;

        std::atomic<uint32_t> width;
        std::atomic<uint32_t> height;
        std::atomic<float> depthValue_center;
        std::atomic<float> depthValue_left;
        std::atomic<float> depthValue_right;

        bool in_air;

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

        void position();
        void avoid();
    };
}