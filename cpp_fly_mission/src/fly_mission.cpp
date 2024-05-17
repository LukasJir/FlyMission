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

        _srvUpload = this->create_service<std_srvs::srv::Trigger>("mission_flier/upload", std::bind(&FlyMission::cbUpload, this, _1, _2));
        _srvTakeOff = this->create_service<std_srvs::srv::Trigger>("mission_flier/take_off", std::bind(&FlyMission::cbTakeOff, this, _1, _2));
        _srvStartMission = this->create_service<std_srvs::srv::Trigger>("mission_flier/start_mission", std::bind(&FlyMission::cbStartMission, this, _1, _2));
        _depthSub = this->create_subscription<sensor_msgs::msg::Image>("mission_flier/depth", 10, std::bind(&FlyMission::cbDepth, this, _1));
    }

    void FlyMission::cbDepth(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        width = msg->width;
        height = msg->height;
        uint32_t x_center = width/2;
        uint32_t y_center = height/2;
        const float* depthData = reinterpret_cast<const float*>(msg->data.data());
        size_t index = y_center * width + x_center;
        depthValue = depthData[index];
    }

    void FlyMission::avoid()
    {
        mavsdk::Mission::MissionItem next_waypoint = mission_items[waypoint];   //nasledujici waypoint
        const float next_waypoint_latitude = next_waypoint.latitude_deg;       //zem. sirka nasledujiciho waypointu
        const float next_waypoint_longitude = next_waypoint.longitude_deg;     //zem. vyska nasledujiciho waypointu

        mavsdk::Mission::MissionItem last_waypoint = mission_items[waypoint-1];     //predchozi waypoint
        const float last_waypoint_latitude = last_waypoint.latitude_deg;           //zem. sirka predchoziho waypointu
        const float last_waypoint_longitude = last_waypoint.longitude_deg;         //zem. vyska predchoziho waypointu

        const auto drone_pos = _telemetry->position();
        const float dron_latitude = drone_pos.latitude_deg;     //zem. sirka dronu
        const float dron_longitude = drone_pos.longitude_deg;   //zem. vyska dronu

        p1 = {last_waypoint_latitude, last_waypoint_longitude};
        p2 = {next_waypoint_latitude, next_waypoint_longitude};
        p_d = {dron_latitude, dron_longitude};

        float citatel = std::fabs((p2[0]-p1[0])*(p_d[1]-p1[1]) - (p_d[0]-p1[0])*(p2[1]-p1[1]));
        float jmenovatel = std::sqrt(std::pow(p2[0]-p1[0],2)+std::pow(p2[1]-p1[1],2));
        float distance_to_line = citatel/jmenovatel;   //vzdalenost dronu od cary

        std::cout << "waypoint:" << waypoint << '\n';
        std::cout << "next_waypoint_latitude:" << next_waypoint_latitude << '\n';
        std::cout << "next_waypoint_longitude:" << next_waypoint_longitude << '\n';
        std::cout << "last_waypoint_latitude:" << last_waypoint_latitude << '\n';
        std::cout << "last_waypoint_longitude:" << last_waypoint_longitude << '\n';
        std::cout << "dron_latitude:" << dron_latitude << '\n';
        std::cout << "dron_longitude:" << dron_longitude << '\n';
        std::cout << "distance_to_line:" << distance_to_line << '\n';
        
        float obstacle_distance = depthValue;   //vzdalenost prekazky (hloubka stredu image)
        //float obstacle_distance = 0.01;
        float obstacle_threshold = 0.1;         //min. vzdalenost prekazky
        
        float sirka = width;
        std::cout << "depthValue:" << obstacle_distance << '\n';
        std::cout << "width(cbDepth):" << sirka << '\n';

/*
        if(obstacle_distance > obstacle_threshold){
            std::cout << "Obstacle detected! Pausing mission.\n";
            const mavsdk::Mission::Result pause_mission_result = _mission.get()->pause_mission();   //pozastaveni mise

            if (pause_mission_result != mavsdk::Mission::Result::Success) {
                std::cerr << "Failed to pause mission:" << pause_mission_result << '\n';
            }
            std::cout << "Mission paused, switching to offboard mode.\n";

            mavsdk::Offboard::Result offboard_result = _offboard.get()->start();     //switch do offboard modu

            if(offboard_result != mavsdk::Offboard::Result::Success) {
                std::cerr << "Offboard start failed: " << offboard_result << '\n';
                return;
            }

            _offboard.get()->set_velocity_body({0.0f, 0.0f, 0.0f, 45.0f});  //otoceni po smeru hodin, 45 stupnu/s
            std::cout << "Avoiding obstacle...\n";
            sleep_for(seconds(2));
            _offboard.get()->set_velocity_body({5.0f, 0.0f, 0.0f, -10.0f}); //let dopredu s otacenim

            sleep_for(seconds(5));  //chvili pockat, aby se dron vzdalil od cary

            if(distance_to_line < 0.1) {   //pokud se dron opet nachazi na puvodni care, znova se pokracuje v misi
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
            }
        }*/
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

        _offboard.get()->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});   //vytvoreni nuloveho setpointu

        mavsdk::Mission::Result start_mission_result = _mission.get()->start_mission();     //start mise
        if (start_mission_result != mavsdk::Mission::Result::Success) {
            std::cerr << "Starting mission failed: " << start_mission_result << '\n';
            //return 1;
        }
        
        while (!_mission.get()->is_mission_finished().second) {
            avoid();
            sleep_for(seconds(1));
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
            37.4122,
            -121.9990,
            14.0f,
            3.0f,
            false,
            -90.0f,
            30.0f,
            mavsdk::Mission::MissionItem::CameraAction::None));

        mission_items.push_back(make_mission_item(
            37.4128,
            -121.9995,
            14.0f,
            10.0f,
            false,
            -90.0f,
            30.0f,
            mavsdk::Mission::MissionItem::CameraAction::None));

        mission_items.push_back(make_mission_item(
            37.4137,
            -121.9993,
            14.0f,
            10.0f,
            false,
            -90.0f,
            30.0f,
            mavsdk::Mission::MissionItem::CameraAction::None));

        mission_items.push_back(make_mission_item(
            37.4125,
            -121.9981,
            14.0f,
            10.0f,
            true,
            -45.0f,
            0.0f,
            mavsdk::Mission::MissionItem::CameraAction::None));

 /*       mission_items.push_back(make_mission_item(
            37.4118,
            -121.9990,
            7.0f,
            10.0f,
            false,
            20.0f,
            60.0f,
            mavsdk::Mission::MissionItem::CameraAction::None));

        mission_items.push_back(make_mission_item(
            37.4116,
            -121.9987,
            25.0f,
            10.0f,
            true,
            0.0f,
            -60.0f,
            mavsdk::Mission::MissionItem::CameraAction::None));

        mission_items.push_back(make_mission_item(
            37.4124,
            -121.9982,
            35.0f,
            10.0f,
            true,
            -45.0f,
            0.0f,
            mavsdk::Mission::MissionItem::CameraAction::None));
            
        mission_items.push_back(make_mission_item(
            37.4127,
            -121.9995,
            25.0f,
            10.0f,
            false,
            -90.0f,
            30.0f,
            mavsdk::Mission::MissionItem::CameraAction::None)); */

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
                    std::cout << "Taking off has finished\n.";
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