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
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <periodic_timer_thread.hpp>
#include <robot_state.hpp>
#include <thread>
#include <yamlloader.hpp>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Exiting loop..." << std::endl;
        keep_running = false;
    }
}

class LeaderArmThread : public PeriodicTimerThread {
public:
    LeaderArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_l,
                    double hz = 500.0)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_l_(control_l) {}

protected:
    void before_start() override { std::cout << "leader (right arm) start thread " << std::endl; }

    void after_stop() override { std::cout << "leader (right arm) stop thread " << std::endl; }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();

        control_l_->bilateral_step();  // 改为双边力反馈控制

        auto now = std::chrono::steady_clock::now();

        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;

        // std::cout << "[Leader] Period: " << elapsed_us << " us" << std::endl;
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control *control_l_;
};

class FollowerArmThread : public PeriodicTimerThread {
public:
    FollowerArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_f,
                      double hz = 500.0)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_f_(control_f) {}

protected:
    void before_start() override { std::cout << "follower (left arm) start thread " << std::endl; }

    void after_stop() override { std::cout << "follower (left arm) stop thread " << std::endl; }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();

        control_f_->bilateral_step();  // 改为双边力反馈控制

        auto now = std::chrono::steady_clock::now();

        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;

        // std::cout << "[Follower] Period: " << elapsed_us << " us" << std::endl;
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control *control_f_;
};

class AdminThread : public PeriodicTimerThread {
public:
    AdminThread(std::shared_ptr<RobotSystemState> leader_state,
                std::shared_ptr<RobotSystemState> follower_state, Control *control_l,
                Control *control_f, bool right_is_leader, double hz = 500.0)
        : PeriodicTimerThread(hz),
          leader_state_(leader_state),
          follower_state_(follower_state),
          control_l_(control_l),
          control_f_(control_f),
          right_is_leader_(right_is_leader) {}

protected:
    void before_start() override { std::cout << "admin start thread " << std::endl; }

    void after_stop() override { std::cout << "admin stop thread " << std::endl; }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();

        // get response
        auto leader_arm_resp = leader_state_->arm_state().get_all_responses();
        auto follower_arm_resp = follower_state_->arm_state().get_all_responses();

        auto leader_hand_resp = leader_state_->hand_state().get_all_responses();
        auto follower_hand_resp = follower_state_->hand_state().get_all_responses();

        // 双向力反馈：主臂参考从臂，从臂参考主臂，均做关节1/7取反
        auto leader_arm_for_follower = leader_arm_resp;
        auto follower_arm_for_leader = follower_arm_resp;
        const std::vector<size_t> reverse_indices = {0, 6};
        for (size_t idx : reverse_indices) {
            if (idx < leader_arm_for_follower.size()) {
                leader_arm_for_follower[idx].position = -leader_arm_for_follower[idx].position;
                leader_arm_for_follower[idx].velocity = -leader_arm_for_follower[idx].velocity;
            }
            if (idx < follower_arm_for_leader.size()) {
                follower_arm_for_leader[idx].position = -follower_arm_for_leader[idx].position;
                follower_arm_for_leader[idx].velocity = -follower_arm_for_leader[idx].velocity;
            }
        }
        // 主臂参考从臂
        leader_state_->arm_state().set_all_references(follower_arm_for_leader);
        leader_state_->hand_state().set_all_references(follower_hand_resp);
        // 从臂参考主臂
        follower_state_->arm_state().set_all_references(leader_arm_for_follower);
        follower_state_->hand_state().set_all_references(leader_hand_resp);

        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;

        // std::cout << "[Admin] Period: " << elapsed_us << " us" << std::endl;
    }

private:
    std::shared_ptr<RobotSystemState> leader_state_;
    std::shared_ptr<RobotSystemState> follower_state_;
    Control *control_l_;
    Control *control_f_;
    bool right_is_leader_;
};

int main(int argc, char **argv) {
    try {
        std::signal(SIGINT, signal_handler);


        // 默认配置：右臂Leader，左臂Follower
        std::string bimanual_urdf_path;
        std::string leader_can_interface = "can0";   // 右臂
        std::string follower_can_interface = "can1"; // 左臂
        bool right_is_leader = true; // 默认右臂为主


        if (argc < 2) {
            std::cerr
                << "Usage: " << argv[0]
                << " <bimanual_urdf_path> [leader_can] [follower_can] [right_is_leader]"
                << std::endl;
            std::cerr << "Example: " << argv[0] << " /tmp/v10_bimanual.urdf can0 can1 1" << std::endl;
            std::cerr << "Default: Leader (right arm) = can0, Follower (left arm) = can1, right_is_leader=1" << std::endl;
            std::cerr << "Set right_is_leader=0 for left arm as leader." << std::endl;
            return 1;
        }

        // Required: URDF path (bimanual robot with both arms)
        bimanual_urdf_path = argv[1];

        // Optional: CAN interfaces
        if (argc >= 4) {
            leader_can_interface = argv[2];
            follower_can_interface = argv[3];
        }
        // Optional: right_is_leader flag
        if (argc >= 5) {
            int flag = std::atoi(argv[4]);
            right_is_leader = (flag != 0);
        }

        // URDF file existence check
        if (!std::filesystem::exists(bimanual_urdf_path)) {
            std::cerr << "[ERROR] Bimanual URDF not found: " << bimanual_urdf_path << std::endl;
            return 1;
        }

        // Setup dynamics
        std::string root_link = "openarm_body_link0";
        
        // Leader (right arm) dynamics
        std::string leader_leaf_link = "openarm_right_hand";
        
        // Follower (left arm) dynamics
        std::string follower_leaf_link = "openarm_left_hand";

        // Output confirmation
        std::cout << "=== OpenArm Bimanual Unilateral Control ===" << std::endl;
        std::cout << "Mode: Right Arm (Leader) <-> Left Arm (Follower)" << std::endl;
        std::cout << "Bimanual URDF path : " << bimanual_urdf_path << std::endl;
        std::cout << "Leader CAN (right)  : " << leader_can_interface << std::endl;
        std::cout << "Follower CAN (left) : " << follower_can_interface << std::endl;
        std::cout << "Root link           : " << root_link << std::endl;
        std::cout << "Leader leaf link    : " << leader_leaf_link << std::endl;
        std::cout << "Follower leaf link  : " << follower_leaf_link << std::endl;

        // 获取可执行文件所在目录，用于定位配置文件
        std::filesystem::path exe_path = std::filesystem::canonical("/proc/self/exe");
        std::filesystem::path exe_dir = exe_path.parent_path();
        // 配置文件在 src/openarm_teleop/config/ 目录下
        std::filesystem::path config_dir = exe_dir.parent_path().parent_path() / "src" / "openarm_teleop" / "config";
        
        std::string leader_config = (config_dir / "leader.yaml").string();
        std::string follower_config = (config_dir / "follower.yaml").string();
        
        std::cout << "Leader config       : " << leader_config << std::endl;
        std::cout << "Follower config     : " << follower_config << std::endl;

        YamlLoader leader_loader(leader_config);
        YamlLoader follower_loader(follower_config);

        // Leader parameters (right arm)
        std::vector<double> leader_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
        std::vector<double> leader_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
        std::vector<double> leader_Fc = leader_loader.get_vector("LeaderArmParam", "Fc");
        std::vector<double> leader_k = leader_loader.get_vector("LeaderArmParam", "k");
        std::vector<double> leader_Fv = leader_loader.get_vector("LeaderArmParam", "Fv");
        std::vector<double> leader_Fo = leader_loader.get_vector("LeaderArmParam", "Fo");

        // Follower parameters (left arm)
        std::vector<double> follower_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
        std::vector<double> follower_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
        std::vector<double> follower_Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
        std::vector<double> follower_k = follower_loader.get_vector("FollowerArmParam", "k");
        std::vector<double> follower_Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
        std::vector<double> follower_Fo = follower_loader.get_vector("FollowerArmParam", "Fo");


        // 根据主从切换，动态分配所有主从相关对象
        Dynamics *leader_arm_dynamics = nullptr;
        Dynamics *follower_arm_dynamics = nullptr;
        openarm::can::socket::OpenArm *leader_openarm = nullptr;
        openarm::can::socket::OpenArm *follower_openarm = nullptr;
        std::shared_ptr<RobotSystemState> leader_state;
        std::shared_ptr<RobotSystemState> follower_state;
        size_t leader_arm_motor_num, follower_arm_motor_num, leader_hand_motor_num, follower_hand_motor_num;

        if (right_is_leader) {
            // 右臂为主
            leader_arm_dynamics = new Dynamics(bimanual_urdf_path, root_link, leader_leaf_link);
            follower_arm_dynamics = new Dynamics(bimanual_urdf_path, root_link, follower_leaf_link);
            leader_arm_dynamics->Init();
            follower_arm_dynamics->Init();
            std::cout << "=== Initializing Leader OpenArm (Right Arm) ===" << std::endl;
            leader_openarm = openarm_init::OpenArmInitializer::initialize_openarm(leader_can_interface, true);
            std::cout << "=== Initializing Follower OpenArm (Left Arm) ===" << std::endl;
            follower_openarm = openarm_init::OpenArmInitializer::initialize_openarm(follower_can_interface, true);
            leader_arm_motor_num = leader_openarm->get_arm().get_motors().size();
            follower_arm_motor_num = follower_openarm->get_arm().get_motors().size();
            leader_hand_motor_num = leader_openarm->get_gripper().get_motors().size();
            follower_hand_motor_num = follower_openarm->get_gripper().get_motors().size();
            leader_state = std::make_shared<RobotSystemState>(leader_arm_motor_num, leader_hand_motor_num);
            follower_state = std::make_shared<RobotSystemState>(follower_arm_motor_num, follower_hand_motor_num);
            leader_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
            leader_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
            leader_Fc = leader_loader.get_vector("LeaderArmParam", "Fc");
            leader_k = leader_loader.get_vector("LeaderArmParam", "k");
            leader_Fv = leader_loader.get_vector("LeaderArmParam", "Fv");
            leader_Fo = leader_loader.get_vector("LeaderArmParam", "Fo");
            follower_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
            follower_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
            follower_Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
            follower_k = follower_loader.get_vector("FollowerArmParam", "k");
            follower_Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
            follower_Fo = follower_loader.get_vector("FollowerArmParam", "Fo");
        } else {
            // 左臂为主
            leader_arm_dynamics = new Dynamics(bimanual_urdf_path, root_link, follower_leaf_link);
            follower_arm_dynamics = new Dynamics(bimanual_urdf_path, root_link, leader_leaf_link);
            leader_arm_dynamics->Init();
            follower_arm_dynamics->Init();
            std::cout << "=== Initializing Leader OpenArm (Left Arm) ===" << std::endl;
            leader_openarm = openarm_init::OpenArmInitializer::initialize_openarm(follower_can_interface, true);
            std::cout << "=== Initializing Follower OpenArm (Right Arm) ===" << std::endl;
            follower_openarm = openarm_init::OpenArmInitializer::initialize_openarm(leader_can_interface, true);
            leader_arm_motor_num = leader_openarm->get_arm().get_motors().size();
            follower_arm_motor_num = follower_openarm->get_arm().get_motors().size();
            leader_hand_motor_num = leader_openarm->get_gripper().get_motors().size();
            follower_hand_motor_num = follower_openarm->get_gripper().get_motors().size();
            leader_state = std::make_shared<RobotSystemState>(leader_arm_motor_num, leader_hand_motor_num);
            follower_state = std::make_shared<RobotSystemState>(follower_arm_motor_num, follower_hand_motor_num);
            // 左臂为主时，主臂用follower.yaml参数，从臂用leader.yaml参数
            leader_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
            leader_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
            leader_Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
            leader_k = follower_loader.get_vector("FollowerArmParam", "k");
            leader_Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
            leader_Fo = follower_loader.get_vector("FollowerArmParam", "Fo");
            follower_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
            follower_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
            follower_Fc = leader_loader.get_vector("LeaderArmParam", "Fc");
            follower_k = leader_loader.get_vector("LeaderArmParam", "k");
            follower_Fv = leader_loader.get_vector("LeaderArmParam", "Fv");
            follower_Fo = leader_loader.get_vector("LeaderArmParam", "Fo");
        }

        std::cout << "Leader arm motor num : " << leader_arm_motor_num << std::endl;
        std::cout << "Follower arm motor num : " << follower_arm_motor_num << std::endl;
        std::cout << "Leader hand motor num : " << leader_hand_motor_num << std::endl;
        std::cout << "Follower hand motor num : " << follower_hand_motor_num << std::endl;

        Control *control_leader = new Control(
            leader_openarm, leader_arm_dynamics, follower_arm_dynamics, leader_state,
            1.0 / FREQUENCY, ROLE_LEADER, right_is_leader ? "right_arm" : "left_arm", leader_arm_motor_num, leader_hand_motor_num);

        Control *control_follower =
            new Control(follower_openarm, leader_arm_dynamics, follower_arm_dynamics,
                        follower_state, 1.0 / FREQUENCY, ROLE_FOLLOWER, right_is_leader ? "left_arm" : "right_arm",
                        follower_arm_motor_num, follower_hand_motor_num);

        control_leader->SetParameter(leader_kp, leader_kd, leader_Fc, leader_k, leader_Fv, leader_Fo);
        control_follower->SetParameter(follower_kp, follower_kd, follower_Fc, follower_k, follower_Fv, follower_Fo);

        // set home position
        std::cout << "=== Adjusting to home position ===" << std::endl;
        std::thread thread_l(&Control::AdjustPosition, control_leader);
        std::thread thread_f(&Control::AdjustPosition, control_follower);
        thread_l.join();
        thread_f.join();
        std::cout << "=== Home position adjustment completed ===" << std::endl;

        // Start control process
        LeaderArmThread leader_thread(leader_state, control_leader, FREQUENCY);
        FollowerArmThread follower_thread(follower_state, control_follower, FREQUENCY);
        AdminThread admin_thread(leader_state, follower_state, control_leader, control_follower,
                     right_is_leader, FREQUENCY);

        std::cout << "=== Starting control threads ===" << std::endl;
        leader_thread.start_thread();
        follower_thread.start_thread();
        admin_thread.start_thread();

        std::cout << "=== Control started: Right arm (Leader) <-> Left arm (Follower) ===" << std::endl;
        std::cout << "Press Ctrl+C to stop..." << std::endl;

        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "=== Stopping control threads ===" << std::endl;
        leader_thread.stop_thread();
        follower_thread.stop_thread();
        admin_thread.stop_thread();

        std::cout << "=== Disabling motors ===" << std::endl;
        leader_openarm->disable_all();
        follower_openarm->disable_all();

        std::cout << "=== Control stopped ===" << std::endl;

    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }

    return 0;
}
