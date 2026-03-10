#include <iostream>
#include <memory>
#include <signal.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <pthread.h>
#include <ctime>
#include <filesystem>
#include <mutex>
#include <algorithm>
#include <thread>
#include <fcntl.h>
#include <sys/select.h>
#include <chrono>
#include <map>
#include <vector>
#include <fstream>

#include <Eigen/Dense>

#include "rcl/Api.h"
#include "rcl/Thread.h"
#include "rcl/JointControl.h"
#include "rcl/Policy.hpp"
#include "rcl/PolicyParams.hpp"

#include "nlohmann/json.hpp"

constexpr float kR2D = 57.295779513f;
constexpr float kD2R = 0.0174532925f;

struct VelocityCommand {
    // Command velocity limits (m/s for linear, rad/s for angular)
    const float kMaxLinVel  = 2.0f;     // Maximum forward/backward velocity
    const float kMaxLatVel  = 1.0f;     // Maximum lateral velocity
    const float kMaxAngVel  = 1.0f;     // Maximum yaw angular velocity
    const float kVelStep    = 0.05f;    // Velocity increment per update (matches keyboard.py)
    const float kVelDecay   = 0.95f;    // Velocity decay factor when keys not pressed (matches keyboard.py)

    std::mutex mtx;
    float vel_x = 0.0f;      // Forward/backward velocity (m/s)
    float vel_y = 0.0f;      // Lateral velocity (m/s)
    float omega_z = 0.0f;    // Yaw angular velocity (rad/s)
    
    void setVelX(float v) {
        std::lock_guard<std::mutex> lock(mtx);
        vel_x = std::clamp(v, -kMaxLinVel, kMaxLinVel);
    }
    
    void setVelY(float v) {
        std::lock_guard<std::mutex> lock(mtx);
        vel_y = std::clamp(v, -kMaxLatVel, kMaxLatVel);
    }
    
    void setOmegaZ(float v) {
        std::lock_guard<std::mutex> lock(mtx);
        omega_z = std::clamp(v, -kMaxAngVel, kMaxAngVel);
    }
    
    void adjustVelX(float delta) {
        std::lock_guard<std::mutex> lock(mtx);
        vel_x = std::clamp(vel_x + delta, -kMaxLinVel, kMaxLinVel);
    }
    
    void adjustVelY(float delta) {
        std::lock_guard<std::mutex> lock(mtx);
        vel_y = std::clamp(vel_y + delta, -kMaxLatVel, kMaxLatVel);
    }
    
    void adjustOmegaZ(float delta) {
        std::lock_guard<std::mutex> lock(mtx);
        omega_z = std::clamp(omega_z + delta, -kMaxAngVel, kMaxAngVel);
    }
    
    void resetVelX() {
        std::lock_guard<std::mutex> lock(mtx);
        vel_x = 0.0f;
    }
    
    void resetVelY() {
        std::lock_guard<std::mutex> lock(mtx);
        vel_y = 0.0f;
    }
    
    void resetOmegaZ() {
        std::lock_guard<std::mutex> lock(mtx);
        omega_z = 0.0f;
    }
    
    void applyDecayX() {
        std::lock_guard<std::mutex> lock(mtx);
        vel_x *= kVelDecay;
    }
    
    void applyDecayY() {
        std::lock_guard<std::mutex> lock(mtx);
        vel_y *= kVelDecay;
    }
    
    void applyDecayZ() {
        std::lock_guard<std::mutex> lock(mtx);
        omega_z *= kVelDecay;
    }
    
    Eigen::Vector3f get() {
        std::lock_guard<std::mutex> lock(mtx);
        return Eigen::Vector3f(vel_x, vel_y, omega_z);
    }
};

JointTable g_jointTable;
std::unique_ptr<JointController> g_jointController;
VelocityCommand g_cmd;
std::string g_policyPath = "";

bool g_isWorking = false;

enum class TaskState {
    Idle = 0,
    Motion,
    PositionLock,
    Control,
};
TaskState g_currentTask = TaskState::Idle;

enum class MotionCmd {
    None = 0,
    MotionSit,
    MotionStand,
    MotionWalk,
};

void signalHandler(int signal) {
    std::cout << "Signal received: " << signal << "\n";
    g_isWorking = false;
    std::_Exit(signal);
}
void goToMotionReady();
void goToMotionGround();
void controlLoop();

void printHelp() {
    std::cout << "\n";
    std::cout << " " << APP_NAME << " Help\n";
    std::cout << "\n -p | --path <path> : Specify the policy path to load\n";
    std::cout << "\n";
    std::cout << " Controls:\n";
    std::cout << "  'z' - Motion Sit\n";
    std::cout << "  'x' - Motion Stand\n";
    std::cout << "  'c' - Motion Walk\n";
    std::cout << " Velocity Control (when in control mode):\n";
    std::cout << "  'w'/'s' - Forward/Backward\n";
    std::cout << "  'a'/'d' - Left/Right\n";
    std::cout << "  'q'/'e' - Yaw Left/Right\n\n";
}

int main(int argc, char* argv[])
{
    signal(SIGTERM, signalHandler);
    signal(SIGINT,  signalHandler);
    signal(SIGHUP,  signalHandler);
    signal(SIGSEGV, signalHandler);
    signal(SIGKILL, signalHandler);
    signal(SIGSEGV, signalHandler);

    std::string path = "";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-p" || arg == "--path") {
            if (i + 1 < argc) {
                path = argv[i + 1];
                ++i;
            } else {
                std::cerr << "Error: Missing value for " << arg << " argument.\n";
                return -1;
            }
        } else if (arg == "-h" || arg == "--help") {
            printHelp();
            return 0;
        }
    }

    std::cout << "Starting " << APP_NAME << "...\n";
    if (getuid() != 0) {
        std::cout << "You are running as NON-ROOT. " << APP_NAME << " will now exit.\n";
        return -1;
    }
    mlockall(MCL_CURRENT|MCL_FUTURE);

    g_isWorking = true;
    try {
        RBQ_API::instance().initialize(23, true);
        RBQ_API::instance().stateEstimation.startEstimation();
        RBQ_API::instance().imu.getQuaternion();
        g_jointController = std::make_unique<JointController>(kMaxJoint);
        g_jointController->syncReferenceToRobot();
        if (path.empty()) {
            std::filesystem::path exe_path = std::filesystem::canonical("/proc/self/exe").parent_path();
            path = (exe_path.lexically_normal().string() + "/../../rbq_gym/policy/rbq10");
        }
        g_policyPath = path;
        std::cout << "Loading policy from: " << g_policyPath << std::endl;
        std::cout << "Initialization complete.\n";
    } catch (const std::exception& e) {
        std::cerr << "Initialization failed: " << e.what() << "\n";
        return -1;
    }
    std::thread controlThread = std::thread(controlLoop);

    // initialize terminal keyboard
    struct termios g_oldTio, g_newTio;
    tcgetattr(STDIN_FILENO, &g_oldTio);
    g_newTio = g_oldTio;
    g_newTio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &g_newTio);
    // Make stdin non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    constexpr int KEY_TIMEOUT_MS = 300;
    std::map<char, std::chrono::steady_clock::time_point> key_last_seen;
    auto get_key_state = [&](char k) -> bool {
        auto it = key_last_seen.find(k);
        if (it == key_last_seen.end()) return false;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count();
        return elapsed < KEY_TIMEOUT_MS;
    };

    while (g_isWorking && g_jointController) {
        MotionCmd command = MotionCmd::None;

        fd_set readfds;
        struct timeval timeout;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;
        
        bool has_input = (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0);

        char key = 0;
        std::vector<char> keys_read;
        while (has_input && read(STDIN_FILENO, &key, 1) == 1) {
            keys_read.push_back(key);
            // Check if more input is available immediately (non-blocking)
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);
            timeout.tv_sec = 0;
            timeout.tv_usec = 0;
            has_input = (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0);
        }

        auto now = std::chrono::steady_clock::now();
        for (char k : keys_read) {
            if (g_currentTask == TaskState::Control) {
                // Update timestamp for velocity control keys
                if (k == 'w' || k == 's' || k == 'a' || k == 'd' || k == 'q' || k == 'e') {
                    key_last_seen[k] = now;
                }
            }
            switch (k) {
            case 'z': command = MotionCmd::MotionSit; break;
            case 'x': command = MotionCmd::MotionStand; break;
            case 'c': command = MotionCmd::MotionWalk; break;
            }
        }

        if (g_currentTask == TaskState::Control) {
            bool key_w = get_key_state('w');
            bool key_s = get_key_state('s');
            bool key_a = get_key_state('a');
            bool key_d = get_key_state('d');
            bool key_q = get_key_state('q');
            bool key_e = get_key_state('e');
            
            // Update velocities based on key presses (similar to keyboard.py)
            // lin_vel_x (forward/backward)
            if (key_w && !key_s) {
                g_cmd.adjustVelX(g_cmd.kVelStep);
            }
            if (key_s && !key_w) {
                g_cmd.adjustVelX(-g_cmd.kVelStep);
            }
            
            // lin_vel_y (lateral left/right)
            if (key_a && !key_d) {
                g_cmd.adjustVelY(g_cmd.kVelStep);
            }
            if (key_d && !key_a) {
                g_cmd.adjustVelY(-g_cmd.kVelStep);
            }
            
            // ang_vel_yaw (yaw rotation)
            if (key_q && !key_e) {
                g_cmd.adjustOmegaZ(g_cmd.kVelStep);
            }
            if (key_e && !key_q) {
                g_cmd.adjustOmegaZ(-g_cmd.kVelStep);
            }
            
            // Apply decay when keys are not pressed (similar to keyboard.py)
            if (!key_w && !key_s) {
                g_cmd.applyDecayX();
            }
            if (!key_a && !key_d) {
                g_cmd.applyDecayY();
            }
            if (!key_q && !key_e) {
                g_cmd.applyDecayZ();
            }
        } else {
            key_last_seen.clear();
        }
        
        switch (command) {
        case MotionCmd::MotionStand: {
            std::cout << "Executing Motion Ready...\n";
            g_currentTask = TaskState::Motion;
            goToMotionReady();
            g_currentTask = TaskState::Idle;
            break;
        }
        case MotionCmd::MotionSit: {
            std::cout << "Executing Motion Ground...\n";
            g_currentTask = TaskState::Motion;
            goToMotionGround();
            g_currentTask = TaskState::Idle;
            break;
        }
        case MotionCmd::MotionWalk: {
            std::cout << "Executing Motion Walk...\n";
            g_currentTask = TaskState::Motion;
            g_jointController->syncReferenceToRobot();
            g_jointController->setAllOwners();
            // Reset velocity command when starting control
            g_cmd.setVelX(0.0f);
            g_cmd.setVelY(0.0f);
            g_cmd.setOmegaZ(0.0f);
            g_currentTask = TaskState::Control;
            break;
        }
        default:
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Shutting down...\n";

    // restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &g_oldTio);

    return 0;
}

void controlLoop()
{
    std::cout << "Control loop started.\n";

    usleep(100 * 1000);

    timespec timeNext;
    clock_gettime(CLOCK_REALTIME, &timeNext);

    bool policyReset = true;
    TaskState lastTask = TaskState::Idle;
    PolicyParams params;
    Policy policy;

    while (g_isWorking && g_jointController) {
        if (lastTask != g_currentTask) {
            lastTask = g_currentTask;
            policyReset = true;
        }
        switch (g_currentTask) {
        case TaskState::Idle: {
            break;
        }
        case TaskState::Motion: {
            g_jointController->updateAllJoints();
            g_jointController->sendReferencesToRobot();
            break;
        }
        case TaskState::Control: {
            static bool policyError = false;
            if (policyError) {
                std::cerr << "Exiting control task due to policy error.\n";
                g_currentTask = TaskState::Idle;
                break;
            }
            // -- 100Hz policy control
            static int decimation_cnt = 0;
            if (decimation_cnt == 5) {
                decimation_cnt = 0;
                try {
                    if (!policy.loaded() || !params.loaded() || policyReset ) {
                        std::cout << "Loading policy from: " << g_policyPath << std::endl;
                        params.loadFromPath(g_policyPath);
                        policy = Policy();
                        policy.reload(g_policyPath);
                        policyReset = false;
                    }
                    if (policy.error() == Policy::ERROR_NONE && policy.loaded() &&
                        params.error() == PolicyParams::ERROR_NONE && params.loaded()) {
                        const int jointSize = 12;
                        Eigen::Vector3f gyro            = RBQ_API::instance().imu.getGyro();
                        Eigen::Quaternion<float> quat   = RBQ_API::instance().imu.getQuaternion();
                        Eigen::VectorXf pos = Eigen::VectorXf::Zero(jointSize);
                        Eigen::VectorXf vel = Eigen::VectorXf::Zero(jointSize);
                        for (int i=0; i<jointSize; i++) {
                            pos[i]    = RBQ_API::instance().joint.getPos(i);
                            vel[i]    = RBQ_API::instance().joint.getVel(i);
                        }
                        Eigen::Vector3f command = g_cmd.get();
                        static std::vector<float> lastActions(jointSize, 0.0f);
                        std::vector<float> obs;
                        for (int i = 0; i < 3; i++)
                            obs.push_back(gyro[i] * params.obs_ang_vel_scale);
                        Eigen::Vector3f proj_grav = quat.inverse() * Eigen::Vector3f(0.0f, 0.0f, -1.0f);
                        for (int i = 0; i < 3; i++)
                            obs.push_back(proj_grav[i]);
                        static float commands_scale[] = {params.obs_lin_vel_scale, params.obs_lin_vel_scale, params.obs_ang_vel_scale};
                        for (int i = 0; i < 3; i++)
                            obs.push_back(command[i] * commands_scale[i]);
                        for (int i = 0; i < jointSize; i++)
                            obs.push_back((pos[i] - params.default_joint_angles[i]) * params.obs_dof_pos_scale);
                        for (int i = 0; i < jointSize; i++)
                            obs.push_back(vel[i] * params.obs_dof_vel_scale);
                        for (int i = 0; i < jointSize; i++)
                            obs.push_back(lastActions[i]);
                        int error;
                        std::vector<float> actions = policy.inference(obs, params.num_observations, params.num_actions, error);
                        for (int i=0; i<jointSize; i++) {
                            actions[i] = std::clamp(actions[i], -params.clip_actions, params.clip_actions);
                            lastActions[i] = actions[i];
                        }
                        if (error == Policy::ERROR_NONE) {
                            for (int i=0; i<jointSize; i++) {
                                const float posRef = actions[i] * params.action_scale + params.default_joint_angles[i];
                                RBQ_API::instance().joint.setPosRef   (i, posRef);
                                RBQ_API::instance().joint.setGainKpRef(i, params.KP[i]);
                                RBQ_API::instance().joint.setGainKdRef(i, params.KD[i]);
                                RBQ_API::instance().joint.setTorqueRef(i, 0);
                            }
                            RBQ_API::instance().joint.setAllJointRef();
                        } else {
                            std::cerr << "Policy error occurred: " << static_cast<int>(policy.error()) << std::endl;
                        }
                    } else {
                        std::cerr << "Policy not loaded or has error.\n";
                        policyError = true;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Policy execution error: " << e.what() << std::endl;
                    policyError = true;
                } catch (...) {
                    std::cerr << "Unknown policy execution error" << std::endl;
                    policyError = true;
                }
            }
            decimation_cnt++;
            break;
        }
        default:
            break;
        }

        Thread::timespec_add_us(&timeNext, kControlPeriodMs * 1000);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &timeNext, NULL);
        clock_gettime(CLOCK_REALTIME, &timeNext);
    }
    std::cout << "Control loop exiting.\n";
}

void goToMotionReady()
{
    if (!g_jointController) return;
    g_jointController->syncReferenceToRobot();
    g_jointController->setAllOwners();

    for (int i = 0; i < kMaxJoint; ++i) {
        RBQ_API::instance().joint.setGainKpRef(i, 200.0f);
        RBQ_API::instance().joint.setGainKdRef(i, 2.5f);
    }
    usleep(500 * 1000);

    float motionTime = 1400.0f;
    float pitchAngles[4];
    pitchAngles[0] = RBQ_API::instance().joint.getPosRef(RBQ_API::Joint::JointID::HRP);
    pitchAngles[1] = RBQ_API::instance().joint.getPosRef(RBQ_API::Joint::JointID::HLP);
    pitchAngles[2] = RBQ_API::instance().joint.getPosRef(RBQ_API::Joint::JointID::FRP);
    pitchAngles[3] = RBQ_API::instance().joint.getPosRef(RBQ_API::Joint::JointID::FLP);

    bool isGrounded = (pitchAngles[0] > 60 * kD2R && pitchAngles[1] > 60 * kD2R &&
                       pitchAngles[2] > 60 * kD2R && pitchAngles[3] > 60 * kD2R);

    const auto& table = g_jointTable;
    const auto& jointCmd = isGrounded ? table.folding : table.ready;

    for (int i = 0; i < kMaxJoint; ++i)
        g_jointController->moveJoint(i, jointCmd[i] * kD2R, motionTime, MoveCommandMode::Absolute);
    usleep((motionTime + 100) * 1000);

    if (isGrounded) {
        for (int i = 0; i < kMaxJoint; ++i)
            g_jointController->moveJoint(i, table.ready[i] * kD2R, motionTime, MoveCommandMode::Absolute);
        usleep((motionTime + 100) * 1000);
    }
}

void goToMotionGround()
{
    if (!g_jointController) return;
    g_jointController->syncReferenceToRobot();
    g_jointController->setAllOwners();

    for (int i = 0; i < kMaxJoint; ++i) {
        RBQ_API::instance().joint.setGainKpRef(i, 200.0f);
        RBQ_API::instance().joint.setGainKdRef(i, 2.5f);
    }

    float motionTime = 2400.0f;
    for (int i = 0; i < kMaxJoint; ++i)
        g_jointController->moveJoint(i, g_jointTable.folding[i] * kD2R, motionTime, MoveCommandMode::Absolute);
    usleep((motionTime + 100) * 1000);

    for (int i = 0; i < kMaxJoint; ++i)
        g_jointController->moveJoint(i, g_jointTable.ground[i] * kD2R, motionTime, MoveCommandMode::Absolute);
    usleep((motionTime + 100) * 1000);
}
