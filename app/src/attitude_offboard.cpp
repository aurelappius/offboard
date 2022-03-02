//
// Does Offboard control using attitude commands.
//
// returns true if everything went well in Offboard control.
//
bool offb_ctrl_attitude(mavsdk::Offboard& offboard)
{
    std::cout << "Starting Offboard attitude control\n";

    // Send it once before starting offboard, otherwise it will be rejected.
    Offboard::Attitude roll{};
    roll.roll_deg = 30.0f;
    roll.thrust_value = 0.6f;
    offboard.set_attitude(roll);

    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard started\n";

    std::cout << "Roll 30 degrees to the right\n";
    offboard.set_attitude(roll);
    sleep_for(seconds(2));

    std::cout << "Stay horizontal\n";
    roll.roll_deg = 0.0f;
    offboard.set_attitude(roll);
    sleep_for(seconds(1));

    std::cout << "Roll 30 degrees to the left\n";
    roll.roll_deg = -30.0f;
    offboard.set_attitude(roll);
    sleep_for(seconds(2));

    std::cout << "Stay horizontal\n";
    roll.roll_deg = 0.0f;
    offboard.set_attitude(roll);
    sleep_for(seconds(2));

    offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard stopped\n";

    return true;
}
