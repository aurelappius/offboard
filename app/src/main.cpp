#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

//FastDDS
#include "sub_callback.h"
#include "domain_participant.h"
#include "subscriber.h"
//MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

//
// Does Offboard control using NED co-ordinates.
//
// returns true if everything went well in Offboard control
//
bool offb_ctrl_ned(mavsdk::Offboard& offboard, DDSSubscriber<idl_msg::MocapPubSubType,cpp_msg::Mocap> &mocap)
{
    std::cout << "Starting Offboard velocity control in NED coordinates\n";

    // Send it once before starting offboard, otherwise it will be rejected.
    const Offboard::VelocityNedYaw stay{};
    offboard.set_velocity_ned(stay);

    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
        return false;
    }
    
    // float x_ref = 0.0;
    // float y_ref = 1.0;
    // float z_ref = 1.5;
    // Offboard::PositionNedYaw msg;
    // msg.down_m=0;
    // msg.east_m=0;
    // msg.north_m=0;
    // while(true){
    //     mocap.listener->wait_for_data();
    //     msg.east_m = x_ref - sub::mocap_msg.pose.position.x;
    //     msg.north_m = y_ref - sub::mocap_msg.pose.position.y;
    //     msg.down_m = -( z_ref - sub::mocap_msg.pose.position.z );

    //     offboard.set_position_ned(msg);

    // }    

    offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard stopped\n";

    return true;
}

//fastDDS test function
void test(DDSSubscriber<idl_msg::MocapPubSubType,cpp_msg::Mocap> &mocap){
    for(int i=0; i<10; i++){
        mocap.listener->wait_for_data();
        std::cout<<"x= "<<sub::mocap_msg.pose.position.x<<std::endl;
    }
}

bool offb_ctrl_attitude(mavsdk::Offboard& offboard, DDSSubscriber<idl_msg::MocapPubSubType,cpp_msg::Mocap> &mocap)
{
    std::cout << "Starting Offboard attitude control\n";

    // Send it once before starting offboard, otherwise it will be rejected.
    Offboard::Attitude stay{};
    stay.roll_deg = 0.0f;
    stay.pitch_deg = 0.0f;
    stay.yaw_deg = 0.0f;
    stay.thrust_value = 0.3f;
    offboard.set_attitude(stay);

    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard started\n";
    
    //params
    float hover_thrust=0;
    float amp = 0.2;

    //keep altitidue
    float z_ref = 0.5;
    float P=0.25;
    float I=0.005;
    float i_ctrl = 0;
    float p_ctrl = 0;
    float T_max = 0.6;
    float T_min = 0.0;

    int T = 10;
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now() + seconds(T);


    while(std::chrono::steady_clock::now()  < end){
        mocap.listener->wait_for_data();
        p_ctrl=z_ref-sub::mocap_msg.pose.position.z;
        i_ctrl+=p_ctrl;
        float thrust = P*p_ctrl + I*i_ctrl;
        //saturation
        if(thrust>T_max){thrust=T_max;}
        if(thrust<T_min){thrust=T_min;}
        stay.thrust_value = thrust;
        hover_thrust=thrust;
        offboard.set_attitude(stay);
    }
    
    std::cout<<hover_thrust<<std::endl;

    //landing
    std::cout<<"landing..."<<std::endl;
    stay.thrust_value=0.25;
    offboard.set_attitude(stay);
    sleep_for(seconds(5));
    stay.thrust_value=0.0;
    offboard.set_attitude(stay);
    sleep_for(seconds(5));
    std::cout<<"landed"<<std::endl;

    offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard stopped\n";

    return true;
}

int main(int argc, char** argv)
{   
    ////fast DDS
    // Create participant. Arguments-> Domain id, QOS name
    DefaultParticipant dp(0, "mocap_subscriber");

    // Create publisher with msg type
    DDSSubscriber mocap_sub(idl_msg::MocapPubSubType(), &sub::mocap_msg, "mocap_srl_quad", dp.participant());


    //activate mocap
    for(int i=0; i<10; i++){
        mocap_sub.listener->wait_for_data();
        std::cout<<"x= "<<sub::mocap_msg.pose.position.x<<std::endl;
    }
    // std::cout<<"starting fastDDS test"<<std::endl;
    // test(mocap_sub);

    ///MAVSDK
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system) {
        return 1;
    }

    // Instantiate plugins.
    auto action = Action{system};
    auto offboard = Offboard{system};
    auto telemetry = Telemetry{system};

    // while (!telemetry.health_all_ok()) {
    //     std::cout << "Waiting for system to be ready\n";
    //     sleep_for(seconds(1));
    // }
    std::cout << "System is ready\n";

    const auto arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }
    std::cout << "Armed\n";


    offb_ctrl_attitude(offboard,mocap_sub);

    const auto disarm_result = action.disarm();
    if (disarm_result != Action::Result::Success) {
        std::cerr << "Disarming failed: " << disarm_result << '\n';
        return 1;
    }
    std::cout << "Disarmed\n";

    // const auto takeoff_result = action.takeoff();
    // if (takeoff_result != Action::Result::Success) {
    //     std::cerr << "Takeoff failed: " << takeoff_result << '\n';
    //     return 1;
    // }
    // std::cerr << "Takeoff failed: " << takeoff_result << '\n';

    // auto in_air_promise = std::promise<void>{};
    // auto in_air_future = in_air_promise.get_future();
    // telemetry.subscribe_landed_state([&telemetry, &in_air_promise](Telemetry::LandedState state) {
    //     if (state == Telemetry::LandedState::InAir) {
    //         std::cout << "Taking off has finished\n.";
    //         telemetry.subscribe_landed_state(nullptr);
    //         in_air_promise.set_value();
    //     }
    // });
    // in_air_future.wait_for(seconds(10));
    // if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
    //     std::cerr << "Takeoff timed out.\n";
    //     return 1;
    // }


    
    // //  using local NED co-ordinates
    // if (!offb_ctrl_ned(offboard,mocap_sub)) {
    //     return 1;
    // }

    // const auto land_result = action.land();
    // if (land_result != Action::Result::Success) {
    //     std::cerr << "Landing failed: " << land_result << '\n';
    //     return 1;
    // }

    // // Check if vehicle is still in air
    // while (telemetry.in_air()) {
    //     std::cout << "Vehicle is landing...\n";
    //     sleep_for(seconds(1));
    // }
    // std::cout << "Landed!\n";

    // // We are relying on auto-disarming but let's keep watching the telemetry for
    // // a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished...\n";

    return 0;
}
