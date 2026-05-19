// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <controller/dynamics.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <periodic_timer_thread.hpp>
#include <thread>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Exiting loop..." << std::endl;
        keep_running = false;
    }
}

class GravityCompensationThread : public PeriodicTimerThread {
public:
    GravityCompensationThread(Dynamics* dynamics, openarm::can::socket::OpenArm* openarm,
                              double hz = 500.0)
        : PeriodicTimerThread(hz), dynamics_(dynamics), openarm_(openarm), target_frequency_(hz) {
        // Pre-allocate vectors to avoid reallocation
        size_t arm_motor_count = openarm_->get_arm().get_motors().size();
        arm_joint_positions_.resize(arm_motor_count, 0.0);
        grav_torques_.resize(arm_motor_count, 0.0);
        cmds_.reserve(arm_motor_count);
    }

protected:
    void before_start() override {
        std::cout << "[INFO] Starting gravity compensation control loop at " << target_frequency_
                  << " Hz" << std::endl;
    }

    void after_stop() override {
        std::cout << "[INFO] Gravity compensation control loop stopped" << std::endl;
    }

    void on_timer() override {
        static int frame_count = 0;
        static auto start_time = std::chrono::steady_clock::now();
        static auto last_hz_display = start_time;

        // Read motor positions
        auto motors = openarm_->get_arm().get_motors();
        for (size_t i = 0; i < motors.size() && i < arm_joint_positions_.size(); ++i) {
            arm_joint_positions_[i] = motors[i].get_position();
        }

        // Calculate gravity compensation
        dynamics_->GetGravity(arm_joint_positions_.data(), grav_torques_.data());

        // Prepare commands (reuse vector to avoid reallocation)
        cmds_.clear();
        for (size_t i = 0; i < grav_torques_.size(); ++i) {
            cmds_.emplace_back(openarm::damiao_motor::MITParam{0, 0, 0, 0, grav_torques_[i]});
        }

        // Send control commands
        openarm_->get_arm().mit_control_all(cmds_);

        // Receive CAN messages with reduced timeout (100 us instead of default 500 us)
        openarm_->recv_all(100);

        // Display frequency statistics every second
        frame_count++;
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_last_display = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          current_time - last_hz_display)
                                          .count();
        if (time_since_last_display >= 1000) {
            auto total_elapsed_us =
                std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time)
                    .count();
            double avg_period_us = total_elapsed_us / static_cast<double>(frame_count);
            double actual_hz = 1e6 / avg_period_us;
            std::cout << "=== Loop Frequency: " << actual_hz << " Hz (target: "
                      << target_frequency_ << " Hz) ===" << std::endl;
            std::cout << "    Average period: " << avg_period_us << " us" << std::endl;
            std::cout << "    Loop execution: " << get_elapsed_time_us() << " us" << std::endl;

            start_time = current_time;
            last_hz_display = current_time;
            frame_count = 0;
        }
    }

private:
    Dynamics* dynamics_;
    openarm::can::socket::OpenArm* openarm_;
    double target_frequency_;
    std::vector<double> arm_joint_positions_;
    std::vector<double> grav_torques_;
    std::vector<openarm::damiao_motor::MITParam> cmds_;
};

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, signal_handler);

        std::string arm_side = "right_arm";
        std::string can_interface = "can0";

        if (argc < 4) {
            std::cerr << "Usage: " << argv[0] << " <arm_side> <can_interface> <urdf_path>"
                      << std::endl;
            std::cerr << "Example: " << argv[0] << " right_arm can0 /tmp/v10_bimanual.urdf"
                      << std::endl;
            return 1;
        }

        arm_side = argv[1];
        can_interface = argv[2];
        std::string urdf_path = argv[3];

        if (arm_side != "left_arm" && arm_side != "right_arm") {
            std::cerr << "[ERROR] Invalid arm_side: " << arm_side
                      << ". Must be 'left_arm' or 'right_arm'." << std::endl;
            return 1;
        }

        if (!std::filesystem::exists(urdf_path)) {
            std::cerr << "[ERROR] URDF file not found: " << urdf_path << std::endl;
            return 1;
        }

        std::cout << "=== OpenArm Gravity Compensation ===" << std::endl;
        std::cout << "Arm side       : " << arm_side << std::endl;
        std::cout << "CAN interface  : " << can_interface << std::endl;
        std::cout << "URDF path      : " << urdf_path << std::endl;

        std::string root_link = "openarm_body_link0";
        std::string leaf_link =
            (arm_side == "left_arm") ? "openarm_left_hand" : "openarm_right_hand";

        Dynamics arm_dynamics(urdf_path, root_link, leaf_link);
        arm_dynamics.Init();

        std::cout << "=== Initializing Leader OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm* openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(can_interface, true);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Create and start gravity compensation control thread
        // Default frequency: 1000 Hz (1ms period) - optimized for 5Mbps CAN-FD
        // For 5Mbps CAN-FD, bandwidth calculation:
        // - 7 arm motors: 7 send + 7 receive = 14 frames per cycle
        // - At 1000Hz: 14,000 frames/sec
        // - Each frame ~20 bytes: 14,000 * 20 * 8 = 2.24 Mbps (well within 5Mbps limit)
        double control_frequency = 1000.0;  // Hz (1ms period)
        GravityCompensationThread control_thread(&arm_dynamics, openarm, control_frequency);

        std::cout << "[INFO] Starting gravity compensation..." << std::endl;
        control_thread.start_thread();

        // Main thread: wait for signal to stop
        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "[INFO] Stopping gravity compensation..." << std::endl;
        control_thread.stop_thread();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        openarm->disable_all();
        openarm->recv_all();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

