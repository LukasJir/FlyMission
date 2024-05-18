#include "cpp_fly_mission/fly_mission.hpp" 

using namespace std::placeholders;
using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

mavsdk::Mission::MissionItem make_mission_item(
    double latitude_deg,  //zemepisna sirka ve stupnich
    double longitude_deg,  //zemepisna vyska ve stupnich
    float relative_altitude_m,  //nadmorska vyska relativni k vysce takeoffu, metry
    float speed_m_s,  //rychlost k dalsimu checkpointu
    bool is_fly_through,  //true = proleti bez zastaveni, false = zastavi na waypointu
    float gimbal_pitch_deg,  //"stoupani" gimbalu ve stupnich
    float gimbal_yaw_deg,  //"natoceni" gimbalu ve stupnich
    Mission::MissionItem::CameraAction camera_action)
{
    Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    new_item.gimbal_pitch_deg = gimbal_pitch_deg;
    new_item.gimbal_yaw_deg = gimbal_yaw_deg;
    new_item.camera_action = camera_action;
    return new_item;
}

namespace mission
{
    FlyMission::FlyMission() : Node("FlyMission")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        //declare_parameter("connection", "udp://:14541");

        _mavsdk = std::make_unique<mavsdk::Mavsdk>();

        //auto connection = get_parameter("udp://:14541").as_string();

        mavsdk::ConnectionResult connectionResult = _mavsdk.get()->add_any_connection("udp://:14541");

        if(connectionResult != mavsdk::ConnectionResult::Success)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection failed");

            throw std::runtime_error("Connection failed");
        }

        auto system = getSystem(*_mavsdk);

        if(system == nullptr)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timed out waiting for system");

            throw std::runtime_error("Timed out waiting for system");
        }

        _action = std::make_unique<mavsdk::Action>(system);
        _telemetry = std::make_shared<mavsdk::Telemetry>(system);
        _mission = std::make_unique<mavsdk::Mission>(system);
        _offboard = std::make_unique<mavsdk::Offboard>(system);
        //_image = std::make_shared<sensor_msgs::msg::Image>();

        _srvUpload = this->create_service<std_srvs::srv::Trigger>("mission_flier/upload", std::bind(&FlyMission::cbUpload, this, _1, _2));
        _srvTakeOff = this->create_service<std_srvs::srv::Trigger>("mission_flier/take_off", std::bind(&FlyMission::cbTakeOff, this, _1, _2));
        _srvStartMission = this->create_service<std_srvs::srv::Trigger>("mission_flier/start_mission", std::bind(&FlyMission::cbStartMission, this, _1, _2));
        _depthSub = this->create_subscription<sensor_msgs::msg::Image>("mission_flier/depth", 10, std::bind(&FlyMission::cbDepth, this, _1));
        _depthPub = this->create_publisher<sensor_msgs::msg::Image>("mission_flier/depth", 10);
        _pathPub = this->create_publisher<nav_msgs::msg::Path>("mission_flier/path", 10); 
        
        //_timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&FlyMission::position, this));
    }

    void FlyMission::publishDepth()
    {
        auto message = sensor_msgs::msg::Image();
        message.height = 480;
        message.width = 640;
        message.encoding = "32FC1";
        message.step = 2560;
        message.data.resize(640 * 480);
        RCLCPP_INFO(this->get_logger(), "Publishing depth image...");
        _depthPub->publish(message);
    }
    
    void FlyMission::publishPath()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing path...");
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "my_frame";

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->get_clock()->now();
        pose_stamped.header.frame_id = "my_frame";
/*
        for (int i = 0; i < 10; ++i)
        {
            pose_stamped.pose.position.x = 0.0;
            pose_stamped.pose.position.y = 0.0;
            pose_stamped.pose.position.z = i;
            pose_stamped.pose.orientation.w = 1.0;
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            path_msg.poses.push_back(pose_stamped);
        }

        _pathPub->publish(path_msg); */

        pose_stamped.pose.position.x = x_d + 2681500;
        pose_stamped.pose.position.y = y_d + 4291460;
        pose_stamped.pose.position.z = 1.0;

        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;

        path_msg.poses.push_back(pose_stamped);

        _pathPub->publish(path_msg); 
    }

    void FlyMission::cbDepth(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Received depth image with height: %d, width: %d", msg->height, msg->width);
        width = msg->width;
        height = msg->height;
        uint32_t x_center = width/2;
        uint32_t y_center = height/2;
        const float* depthData = reinterpret_cast<const float*>(msg->data.data());
        size_t index = y_center * width + x_center;
        depthValue = depthData[index];
    }

    void FlyMission::position()
    {
        std::cout << "waypoint:" << waypoint << '\n';
        //std::cout << "drone_start_latitude:" << drone_start_pos.latitude_deg << '\n';
        //std::cout << "drone_start_longitude:" << drone_start_pos.longitude_deg << '\n';
        
        mavsdk::Mission::MissionItem next_waypoint = mission_items[waypoint];   //nasledujici waypoint
        next_waypoint_latitude = next_waypoint.latitude_deg;                    //zem. sirka nasledujiciho waypointu
        next_waypoint_longitude = next_waypoint.longitude_deg;                  //zem. vyska nasledujiciho waypointu

        mavsdk::Mission::MissionItem last_waypoint = mission_items[waypoint-1];     //predchozi waypoint       

        if(waypoint == 0){
            last_waypoint_latitude = drone_start_pos.latitude_deg;      //zem. sirka startovni pozice
            last_waypoint_longitude = drone_start_pos.longitude_deg;    //zem. vyska startovni pozice
        }else{
            last_waypoint_latitude = last_waypoint.latitude_deg;        //zem. sirka predchoziho waypointu
            last_waypoint_longitude = last_waypoint.longitude_deg;      //zem. vyska predchoziho waypointu
        }

        drone_pos = _telemetry->position();
        drone_latitude = drone_pos.latitude_deg;     //aktualni zem. sirka dronu
        drone_longitude = drone_pos.longitude_deg;   //aktualni zem. vyska dronu

        p1 = {last_waypoint_latitude*(M_PI/180.0), last_waypoint_longitude*(M_PI/180.0)};
        p2 = {next_waypoint_latitude*(M_PI/180.0), next_waypoint_longitude*(M_PI/180.0)};
        p_d = {drone_latitude*(M_PI/180.0), drone_longitude*(M_PI/180.0)};

        double R = 6371000;
        float x1 = R*std::cos(p1[0])*std::cos(p1[1]);
        float y1 = R*std::cos(p1[0])*std::sin(p1[1]);
        float x2 = R*std::cos(p2[0])*std::cos(p2[1]);
        float y2 = R*std::cos(p2[0])*std::sin(p2[1]);
        x_d = R*std::cos(p_d[0])*std::cos(p_d[1]);
        y_d = R*std::cos(p_d[0])*std::sin(p_d[1]);

        float citatel = std::fabs((x2-x1)*(y_d-y1) - (x_d-x1)*(y2-y1));
        float jmenovatel = std::sqrt(std::pow(x2-x1,2)+std::pow(y2-y1,2));
        distance_to_line = citatel/jmenovatel;            //vzdalenost dronu od cary

        float xLast = R*std::cos(last_waypoint_latitude*(M_PI/180.0))*std::cos(last_waypoint_longitude*(M_PI/180.0));
        float yLast = R*std::cos(drone_latitude*(M_PI/180.0))*std::sin(drone_longitude*(M_PI/180.0));
        distance_from_last_w = std::sqrt(std::pow(x_d-xLast,2)+std::pow(y_d-yLast,2));      //vzdalenost od posledniho waypointu
        std::cout << "distance_from_last_w:" << distance_from_last_w << '\n';

        //std::cout << "next_waypoint_latitude:" << next_waypoint_latitude << '\n';
        //std::cout << "next_waypoint_longitude:" << next_waypoint_longitude << '\n';
        //std::cout << "last_waypoint_latitude:" << last_waypoint_latitude << '\n';
        //std::cout << "last_waypoint_longitude:" << last_waypoint_longitude << '\n';
        //std::cout << "drone_latitude:" << drone_latitude << '\n';
        //std::cout << "drone_x norm:" << x_d + 2681500 << '\n';
        //std::cout << "drone_longitude:" << drone_longitude << '\n';
        //std::cout << "drone_y norm:" << y_d + 4291460 << '\n';
        std::cout << "distance_to_line:" << distance_to_line << '\n';
    }

    void FlyMission::avoid()
    {
        //float obstacle_distance = depthValue;   //vzdalenost prekazky (hloubka stredu image)
        float obstacle_distance = 10.01;
        float obstacle_threshold = 0.1;         //min. vzdalenost prekazky
        float line_threshold = 5;             //vzdalenost od cary pro navrat k misi
        
        float sirka = width;
        //std::cout << "depthValue:" << obstacle_distance << '\n';
        //std::cout << "width(cbDepth):" << sirka << '\n';

        //if(obstacle_distance < obstacle_threshold){
        if(distance_from_last_w > 4.5 && distance_from_last_w < 6.5){          
            std::cout << "Obstacle detected! Pausing mission.\n";
            std::cout << "distance_to_line:" << distance_to_line << '\n';
            const mavsdk::Mission::Result pause_mission_result = _mission.get()->pause_mission();   //pozastaveni mise

            if (pause_mission_result != mavsdk::Mission::Result::Success) {
                std::cerr << "Failed to pause mission:" << pause_mission_result << '\n';
            }
            std::cout << "Mission paused, switching to offboard mode.\n";

            _offboard.get()->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});   //vytvoreni nuloveho setpointu

            mavsdk::Offboard::Result offboard_result = _offboard.get()->start();     //switch do offboard modu

            if(offboard_result != mavsdk::Offboard::Result::Success) {
                std::cerr << "Offboard start failed: " << offboard_result << '\n';
                return;
            }

            _offboard.get()->set_velocity_body({0.0f, 0.0f, 0.0f, 50.0f});  //otoceni po smeru hodin, 50 stupnu/s
            std::cout << "Avoiding obstacle...\n";
            sleep_for(std::chrono::milliseconds(2500));
            _offboard.get()->set_velocity_body({3.0f, 0.0f, 0.0f, -25.0f}); //let dopredu s otacenim

            std::cout << "distance_to_line:" << distance_to_line << '\n';

            sleep_for(seconds(10));  //chvili pockat, aby se dron vzdalil od cary

            std::cout << "Obstacle avoided, returning to original course.\n";
            mavsdk::Offboard::Result offboard_result2 = _offboard.get()->stop();    //switch z offboard modu
            if(offboard_result2 != mavsdk::Offboard::Result::Success) {
                    std::cerr << "Offboard stop failed: " << offboard_result2 << '\n';
                    return;
            }

            const mavsdk::Mission::Result start_mission_again_result = _mission.get()->start_mission();     //znovu spusteni mise
            if (start_mission_again_result != mavsdk::Mission::Result::Success) {
                    std::cerr << "Returning to original course failed: " << start_mission_again_result << '\n';
            }

            /*
            if(distance_to_line < line_threshold) {   //pokud se dron opet nachazi na puvodni care, znova se pokracuje v misi
                std::cout << "distance_to_line:" << distance_to_line << '\n';
                std::cout << "Obstacle avoided, returning to original course.\n";
                mavsdk::Offboard::Result offboard_result = _offboard.get()->stop();    //switch z offboard modu

                if(offboard_result != mavsdk::Offboard::Result::Success) {
                    std::cerr << "Offboard stop failed: " << offboard_result << '\n';
                    return;
                }

                const mavsdk::Mission::Result start_mission_again_result = _mission.get()->start_mission();     //znovu spusteni mise

                if (start_mission_again_result != mavsdk::Mission::Result::Success) {
                    std::cerr << "Returning to original course failed: " << start_mission_again_result << '\n';
                }
            }*/
        }
    }

    void FlyMission::cbStartMission(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse)
    {
        std::atomic<bool> want_to_pause{false};
        // Before starting the mission, we want to be sure to subscribe to the mission progress.
        _mission.get()->subscribe_mission_progress([this, &want_to_pause](mavsdk::Mission::MissionProgress mission_progress) {
            std::cout << "Mission status update: " << mission_progress.current << " / "
                    << mission_progress.total << '\n';

            if (mission_progress.current >= 2) {
                // We can only set a flag here. If we do more request inside the callback,
                // we risk blocking the system.
                want_to_pause = true;
            }
            waypoint = mission_progress.current;    //index nasledujiciho waypointu
        });

        drone_start_pos = _telemetry->position();    //startovni pozice dronu

        mavsdk::Mission::Result start_mission_result = _mission.get()->start_mission();     //start mise
        if (start_mission_result != mavsdk::Mission::Result::Success) {
            std::cerr << "Starting mission failed: " << start_mission_result << '\n';
            //return 1;
        }
        
        while (!_mission.get()->is_mission_finished().second) {
            avoid();
            position();
            //publishPath();
            publishDepth();
            sleep_for(std::chrono::milliseconds(500));
        }
/*
        // We are done, and can do RTL to go home.
        std::cout << "Commanding RTL...\n";
        const Action::Result rtl_result = _action.get()->return_to_launch();
        if (rtl_result != Action::Result::Success) {
            std::cout << "Failed to command RTL: " << rtl_result << '\n';
            //return 1;
        }
        std::cout << "Commanded RTL.\n";
*/
        // We need to wait a bit, otherwise the armed state might not be correct yet.
        sleep_for(seconds(2));

        while (_telemetry.get()->armed()) {
            // Wait until we're done.
            sleep_for(seconds(1));
        }
        std::cout << "Disarmed, exiting.\n";
    }

    void FlyMission::upload()
    {
        std::cout << "Creating and uploading mission\n";

        mission_items.push_back(make_mission_item(
            37.4124,
            -121.9985,
            20.0f,      //14 prekazky, 20 bez
            5.0f,
            false,
            -90.0f,
            30.0f,
            mavsdk::Mission::MissionItem::CameraAction::None));

        mission_items.push_back(make_mission_item(
            37.4128,
            -121.9995,
            20.0f,
            5.0f,
            false,
            -90.0f,
            30.0f,
            mavsdk::Mission::MissionItem::CameraAction::None));

        mission_items.push_back(make_mission_item(
            37.4137,
            -121.9993,
            20.0f,
            5.0f,
            false,
            -90.0f,
            30.0f,
            mavsdk::Mission::MissionItem::CameraAction::None));

        mission_items.push_back(make_mission_item(
            37.4125,
            -121.9981,
            20.0f,
            5.0f,
            true,
            -45.0f,
            0.0f,
            mavsdk::Mission::MissionItem::CameraAction::None));

        mavsdk::Mission::MissionPlan mission_plan{};
        mission_plan.mission_items = mission_items;
        const mavsdk::Mission::Result upload_result = _mission.get()->upload_mission(mission_plan);
    }

    void FlyMission::cbUpload(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse)
    {
        try
        {
            upload();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';

            aResponse->success = false;
            aResponse->message = "Upload failed...";
        }

        aResponse->success = true;
        aResponse->message = "Uploading...";
    }

    void FlyMission::takeOff()
    {
        const auto arm_result = _action.get()->arm();
        if(arm_result != mavsdk::Action::Result::Success)
        {
            std::cerr << "Arming failed: " << arm_result << '\n';
            throw std::runtime_error("Arming error");
        }

        std::cout << "Armed\n";

        const auto takeoff_result = _action.get()->takeoff();

        if(takeoff_result != mavsdk::Action::Result::Success)
        {
            std::cerr << "Takeoff failed: " << takeoff_result << '\n';
            throw std::runtime_error("Takeoff error");
        }

        auto in_air_promise = std::promise<void>{};
        auto in_air_future = in_air_promise.get_future();

        _telemetry.get()->subscribe_landed_state(
            [this, &in_air_promise](mavsdk::Telemetry::LandedState state) {
                if(state == mavsdk::Telemetry::LandedState::InAir)
                {
                    std::cout << "Taking off has finished.\n";
                    _telemetry.get()->subscribe_landed_state(nullptr);
                    in_air_promise.set_value();
                }
            });

        in_air_future.wait_for(std::chrono::seconds(10));
        if(in_air_future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
        {
            std::cerr << "Takeoff timed out.\n";
            throw std::runtime_error("Takeoff timeout error");
        }
    }

    void FlyMission::cbTakeOff(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse)
    {
        try
        {
            takeOff();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';

            aResponse->success = false;
            aResponse->message = "Could not take off...";
        }

        aResponse->success = true;
        aResponse->message = "Flying...";
    }

    std::shared_ptr<mavsdk::System> FlyMission::getSystem(mavsdk::Mavsdk& aMavsdk)
    {
        std::cout << "Waiting to discover system...\n";
        auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
        auto fut = prom.get_future();

        // We wait for new systems to be discovered, once we find one that has an
        // autopilot, we decide to use it.
        aMavsdk.subscribe_on_new_system([&aMavsdk, &prom]() {
            auto system = aMavsdk.systems().back();

            if(system->has_autopilot())
            {
                std::cout << "Discovered autopilot\n";

                // Unsubscribe again as we only want to find one system.
                aMavsdk.subscribe_on_new_system(nullptr);
                prom.set_value(system);
            }
        });

        // We usually receive heartbeats at 1Hz, therefore we should find a
        // system after around 3 seconds max, surely.
        if(fut.wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
        {
            std::cerr << "No autopilot found.\n";
            return nullptr;
        }

        // Get discovered system now.
        return fut.get();
    }
}

int main(int argc, char *argv[])
{
    std::cout << "Starting MissionFlier node..." << std::endl;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<mission::FlyMission>());
    }
    catch(const std::runtime_error& e)
    {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    catch(...)
    {

    }

    rclcpp::shutdown();
    
    return 0;
}