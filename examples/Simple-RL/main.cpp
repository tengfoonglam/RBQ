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
#include <atomic>
#include <algorithm>

#include <Eigen/Dense>

#include "rcl/Api.h"
#include "rcl/Thread.h"
#include "rcl/JointControl.h"
#include "rcl/Policy.hpp"

constexpr float kR2D = 57.295779513f;
constexpr float kD2R = 0.0174532925f;
constexpr long kControlPeriodUs = kControlPeriodMs * 1000;

// Command velocity limits (m/s for linear, rad/s for angular)
constexpr float kMaxLinVel = 1.0f;      // Maximum forward/backward velocity
constexpr float kMaxLatVel = 0.4f;     // Maximum lateral velocity
constexpr float kMaxAngVel = 1.0f;     // Maximum yaw angular velocity
constexpr float kVelStep = 0.1f;       // Velocity increment per key press

struct VelocityCommand {
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
    
    Eigen::Vector3f get() {
        std::lock_guard<std::mutex> lock(mtx);
        return Eigen::Vector3f(vel_x, vel_y, omega_z);
    }
    
    void decay(float factor = 0.95f) {
        std::lock_guard<std::mutex> lock(mtx);
        vel_x *= factor;
        vel_y *= factor;
        omega_z *= factor;
        // Stop if very small
        if (std::abs(vel_x) < 0.01f) vel_x = 0.0f;
        if (std::abs(vel_y) < 0.01f) vel_y = 0.0f;
        if (std::abs(omega_z) < 0.01f) omega_z = 0.0f;
    }
};

JointTable g_jointTable;
std::unique_ptr<JointController> g_jointController;
std::unique_ptr<Policy> g_policy;
VelocityCommand g_velocityCommand;

bool g_isWorking = false;

enum class TaskState {
    Idle = 0,
    Motion,
    PositionLock,
    Control,
};
TaskState g_currentTask = TaskState::Idle;

enum class UserCommand {
    None = 0,
    MotionReady,
    MotionGround,
    MotionWalk,
};

void signalHandler(int signal);
void goToMotionReady();
void goToMotionGround();
void* controlLoop(void*);

int main(int argc, char* argv[])
{
    signal(SIGTERM, signalHandler);
    signal(SIGINT,  signalHandler);
    signal(SIGHUP,  signalHandler);
    signal(SIGSEGV, signalHandler);

    g_isWorking = true;
    std::cout << "Starting RobotControlApp...\n";
    std::cout << "Controls:\n";
    std::cout << "  'x' - MOTION_GROUND\n";
    std::cout << "  'z' - MOTION_READY\n";
    std::cout << "  'c' - Start walking control\n";
    std::cout << "  'w' - Increase forward velocity\n";
    std::cout << "  's' - Increase backward velocity\n";
    std::cout << "  'a' - Increase left velocity\n";
    std::cout << "  'd' - Increase right velocity\n";
    std::cout << "  'q' - Quit\n";

    // initialize terminal keyboard
    struct termios g_oldTio, g_newTio;
    tcgetattr(STDIN_FILENO, &g_oldTio);
    g_newTio = g_oldTio;
    g_newTio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &g_newTio);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "Memory lock failed.\n";
        return 1;
    }

    try {
        RBQ_API::instance().initialize(23, true);
        RBQ_API::instance().stateEstimation.startEstimation();
        RBQ_API::instance().imu.getQuaternion();
        g_jointController = std::make_unique<JointController>(kMaxJoint);
        g_jointController->syncReferenceToRobot();
        std::filesystem::path exe_path = std::filesystem::canonical("/proc/self/exe").parent_path();
        std::string path = (exe_path.lexically_normal().string() + "/../../resources/policy/rbq10_trot");
        std::cout << "Loading policy from: " << path << std::endl;
        g_policy = std::make_unique<Policy>(path);
        std::cout << "Control thread starting...\n";
        pthread_t controlThread;
        if (!Thread::generate_rt_thread(controlThread, controlLoop, "ControlLoop", 1, 90, nullptr)) {
            std::cerr << "Failed to create control thread\n";
            throw(std::exception());
        }
    } catch (const std::exception& e) {
        std::cerr << "Initialization failed: " << e.what() << "\n";
        g_isWorking = false;
    }

    while (g_isWorking && g_jointController) {
        UserCommand command = UserCommand::None;
        char key = 0;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            switch (key) {
            case 'x': command = UserCommand::MotionGround; break;
            case 'z': command = UserCommand::MotionReady; break;
            case 'c': command = UserCommand::MotionWalk; break;
            case 'w': g_velocityCommand.adjustVelX(kVelStep); break;
            case 's': g_velocityCommand.adjustVelX(-kVelStep); break;
            case 'a': g_velocityCommand.adjustVelY(kVelStep); break;
            case 'd': g_velocityCommand.adjustVelY(-kVelStep); break;
            case 'q': g_isWorking = false; break;
            }
        }

        switch (command) {
        case UserCommand::MotionReady: {
            std::cout << "Executing Motion Ready...\n";
            g_currentTask = TaskState::Motion;
            goToMotionReady();
            g_currentTask = TaskState::Idle;
            break;
        }
        case UserCommand::MotionGround: {
            std::cout << "Executing Motion Ground...\n";
            g_currentTask = TaskState::Motion;
            goToMotionGround();
            g_currentTask = TaskState::Idle;
            break;
        }
        case UserCommand::MotionWalk: {
            std::cout << "Executing Motion Walk...\n";
            g_currentTask = TaskState::Motion;
            g_jointController->syncReferenceToRobot();
            g_jointController->setAllOwners();
            // Reset velocity command when starting control
            g_velocityCommand.setVelX(0.0f);
            g_velocityCommand.setVelY(0.0f);
            g_velocityCommand.setOmegaZ(0.0f);
            g_currentTask = TaskState::Control;
            break;
        }
        default:
            break;
        }
    }

    std::cout << "Shutting down...\n";

    // restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &g_oldTio);

    return 0;
}

void* controlLoop(void*)
{
    std::cout << "Control loop started.\n";

    usleep(100 * 1000);

    timespec timeNext;
    clock_gettime(CLOCK_REALTIME, &timeNext);

    while (g_isWorking && g_jointController) {

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
            // Ensure motion ownership is maintained
            static bool ownership_set = false;
            if (!ownership_set) {
                std::cout << "Setting motion ownership in control loop..." << std::endl;
                for (int i = 0; i < 12; i++) {
                    RBQ_API::instance().joint.setMotionOwner(i);
                }
                ownership_set = true;
                std::cout << "Motion ownership set for all joints." << std::endl;
            }

            // Apply velocity decay (gradually reduce velocity when no keys are pressed)
            g_velocityCommand.decay(0.99f);

            // -- 100Hz policy control
            static int decimation_cnt = 0;
            if (decimation_cnt == 5){
                decimation_cnt = 0;
                try {
                    if (g_policy) {
                        Eigen::Vector3f gyro            = RBQ_API::instance().imu.getGyro();
                        Eigen::Quaternion<float> quat   = RBQ_API::instance().imu.getQuaternion();
                        const int jointSize = 12;
                        Eigen::VectorXf pos = Eigen::VectorXf::Zero(jointSize);
                        Eigen::VectorXf vel = Eigen::VectorXf::Zero(jointSize);
                        for (int i=0; i<jointSize; i++) {
                            pos[i]    = RBQ_API::instance().joint.getPos(i);
                            vel[i]    = RBQ_API::instance().joint.getVel(i);
                        }
                        // Get velocity command from keyboard input
                        Eigen::Vector3f command = g_velocityCommand.get();

                        static bool first_run = true;
                        Eigen::MatrixXf actions = g_policy->compute(gyro, quat, pos, vel, command, first_run);
                        first_run = false;
                        if (g_policy->error() == Policy::ERROR_NONE) {
                            for (int i=0; i<jointSize; i++) {
                                RBQ_API::instance().joint.setPosRef   (i, actions(i, 0));
                                RBQ_API::instance().joint.setGainKpRef(i, actions(i, 1));
                                RBQ_API::instance().joint.setGainKdRef(i, actions(i, 2));
                            }
                            RBQ_API::instance().joint.setAllJointRef();
                        } else {
                            std::cerr << "Policy error occurred: " << static_cast<int>(g_policy->error()) << std::endl;
                        }
                    }
                } catch (const std::exception& e) {
                    static bool policy_error_logged = false;
                    if (!policy_error_logged) {
                        std::cerr << "Policy execution error: " << e.what() << std::endl;
                        policy_error_logged = true;
                    }
                } catch (...) {
                    static bool policy_error_logged = false;
                    if (!policy_error_logged) {
                        std::cerr << "Unknown policy execution error" << std::endl;
                        policy_error_logged = true;
                    }
                }
            }
            decimation_cnt++;
            break;
        }
        default:
            break;
        }

        Thread::timespec_add_us(&timeNext, kControlPeriodUs);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &timeNext, NULL);
        clock_gettime(CLOCK_REALTIME, &timeNext);
    }

    std::cout << "Control loop exiting.\n";
    return nullptr;
}

void signalHandler(int signal)
{
    std::cout << "Signal received: " << signal << "\n";
    g_isWorking = false;
    std::_Exit(signal);
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
