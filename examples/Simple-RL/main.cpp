#include <iostream>
#include <memory>
#include <signal.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <pthread.h>
#include <ctime>
#include <Eigen/Dense>

#include "rcl/Api.h"
#include "rcl/Thread.h"
#include "rcl/JointControl.h"

#include "PolicyRunner.h"

constexpr float kR2D = 57.295779513f;
constexpr float kD2R = 0.0174532925f;
constexpr long kControlPeriodUs = kControlPeriodMs * 1000;

JointTable g_jointTable;
std::unique_ptr<JointController> g_jointController;
std::unique_ptr<PolicyRunner> g_policyRunner;

int g_jointJogNo = 0;
int g_jogMoveDir = 0;
bool g_isWorking = false;

struct termios g_oldTio, g_newTio;
bool g_keyboardInitialized = false;

enum class TaskState {
    Idle = 0,
    Motion,
    JogMove,
    PositionLock,
    Control,
};
TaskState g_currentTask = TaskState::Idle;

enum class UserCommand {
    None = 0,
    Test,
    MotionReady,
    MotionGround,
    MotionWalk,
};

void initializeKeyboard();
void cleanupKeyboard();
char readKeyboard();
void signalHandler(int signal);
void cleanupResources();
void goToMotionReady();
void goToMotionGround();
void goRLcontrol();
void* controlLoop(void*);

int main(int argc, char* argv[]) {
    std::cout << "Starting RobotControlApp...\n";
    std::cout << "Press 'x' for MOTION_GROUND, 'z' for MOTION_READY, 'q' to quit.\n";

    initializeKeyboard();

    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);
    signal(SIGHUP, signalHandler);
    signal(SIGSEGV, signalHandler);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "Memory lock failed.\n";
        return 1;
    }

    try {
        RBQ_API::instance().initialize(22, true);
        // RBQ_API::instance().stateEstimation.startEstimation();
        Eigen::Quaternion<float> quat = RBQ_API::instance().imu.getQuaternion();
        Eigen::Vector3f gyro    = RBQ_API::instance().imu.getGyro();
        Eigen::Vector3f acc     = RBQ_API::instance().imu.getAcc();
        Eigen::Vector3f rpy     = RBQ_API::instance().imu.getRPY();
        g_jointController = std::make_unique<JointController>(kMaxJoint);
        g_jointController->syncReferenceToRobot();
        g_policyRunner = std::make_unique<PolicyRunner>();
    } catch (const std::exception& e) {
        std::cerr << "Initialization failed: " << e.what() << "\n";
        cleanupResources();
        return 1;
    }

    std::cout << "Control thread starting...\n";
    pthread_t controlThread;
    if (!Thread::generate_rt_thread(controlThread, controlLoop, "ControlLoop", 1, 90, nullptr)) {
        std::cerr << "Failed to create control thread\n";
        cleanupResources();
        return 1;
    }

    g_isWorking = true;

    while (g_isWorking && g_jointController && g_policyRunner) {
        char key = readKeyboard();
        UserCommand command = UserCommand::None;

        switch (key) {
        case 'x': command = UserCommand::MotionGround; break;
        case 'z': command = UserCommand::MotionReady; break;
        case 'c': command = UserCommand::MotionWalk; break;
        case 'q': g_isWorking = false; break;
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
            float initial_joint_pos[12] = {0,};
            for (int i = 0; i < 12; i++)
                initial_joint_pos[i] = static_cast<float>(g_jointController->getAngle(i));
            g_policyRunner->initialize_observation(initial_joint_pos);
            goRLcontrol();
            // g_currentTask = TaskState::Idle;
            g_currentTask = TaskState::Control;
            break;
        }
        default:
            break;
        }
    }

    std::cout << "Shutting down...\n";
    cleanupKeyboard();
    cleanupResources();
    return 0;
}

void* controlLoop(void*) {
    std::cout << "Control loop started.\n";

    timespec timeNext;
    usleep(100 * 1000);
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
        case TaskState::JogMove: {
            // Add jog logic here if needed
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

            ///-----------Get Motor & Joint Data--------------------------------
            float current_joint_pos[12];
            float current_joint_vel[12];
            float current_joint_torque[12];
            float current_joint_kp[12];
            float current_joint_kd[12];
            for(int i=0; i<12; i++){
                current_joint_pos[i]    = RBQ_API::instance().joint.getPos(i);
                current_joint_vel[i]    = RBQ_API::instance().joint.getVel(i);
                current_joint_torque[i] = RBQ_API::instance().joint.getTorque(i);
                current_joint_kp[i]     = RBQ_API::instance().joint.getGainKp(i);
                current_joint_kd[i]     = RBQ_API::instance().joint.getGainKd(i);
            }

            Eigen::Quaternion<float> quat = RBQ_API::instance().imu.getQuaternion();
            Eigen::Vector3f gyro            = RBQ_API::instance().imu.getGyro();
            Eigen::Vector3f acc             = RBQ_API::instance().imu.getAcc();
            Eigen::Vector3f rpy             = RBQ_API::instance().imu.getRPY();


            //-------------gamepad----------
            // float left_jog_x = 0.0f;
            // float left_jog_y = 0.0f;
            // float left_trigger = 0.0f;
            // float right_jog_x = 0.0f;
            // float right_jog_y = 0.0f;
            // float right_trigger = 0.0f;
            // bool BTN_A = false, BTN_B = false, BTN_X = false, BTN_Y = false;
            // bool BTN_LB = false, BTN_RB = false;
            // bool BTN_BACK = false, BTN_START = false, BTN_LOGI = false;
            // bool BTN_LJOG = false, BTN_RJOG = false;
            // bool ARROW_UP = false, ARROW_DW = false, ARROW_L = false, ARROW_R = false;
            bool gamepad_available = false;
            // try {
            //     if (g_api) {
            //         g_api->gamepad.getLeftJogX(left_jog_x);
            //         g_api->gamepad.getLeftJogY(left_jog_y);
            //         g_api->gamepad.getLeftTrigger(left_trigger);
            //         g_api->gamepad.getRightJogX(right_jog_x);
            //         g_api->gamepad.getRightJogY(right_jog_y);
            //         g_api->gamepad.getRightTrigger(right_trigger);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::A, BTN_A);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::B, BTN_B);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::X, BTN_X);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::Y, BTN_Y);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::LB, BTN_LB);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::RB, BTN_RB);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::BACK, BTN_BACK);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::START, BTN_START);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::LOGITECH, BTN_LOGI);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::LJOG, BTN_LJOG);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::RJOG, BTN_RJOG);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_UP, ARROW_UP);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_DOWN, ARROW_DW);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_LEFT, ARROW_L);
            //         g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_RIGHT, ARROW_R);
            //         gamepad_available = true;
            //     }
            // } catch (const std::exception& e) {
            //     static bool gamepad_error_logged = false;
            //     if (!gamepad_error_logged) {
            //         std::cerr << "Gamepad access error (using defaults): " << e.what() << std::endl;
            //         gamepad_error_logged = true;
            //     }
            //     // Keep default values (all zeros/false)
            // } catch (...) {
            //     static bool gamepad_error_logged = false;
            //     if (!gamepad_error_logged) {
            //         std::cerr << "Unknown gamepad access error (using defaults)" << std::endl;
            //         gamepad_error_logged = true;
            //     }
            // }

            //--------------50Hz policy run-------------------------------------
            static int decimation_cnt = 0;
            if(decimation_cnt == 10){
                decimation_cnt = 0;
                try {
                    if (g_policyRunner) {
                        g_policyRunner->set_base_ang_vel(gyro);
                        g_policyRunner->set_projected_gravity(quat);
                        g_policyRunner->set_joint_pos(current_joint_pos);
                        g_policyRunner->set_joint_vel(current_joint_vel);
                        // Use gamepad commands if available, otherwise use zero commands
                        if (gamepad_available) {
                            g_policyRunner->set_commands(0, 0, 0);  //TODO : check gamepad command
                        } else {
                            g_policyRunner->set_commands(0.0f, 0.0f, 0.0f);
                        }
                        auto policy_result = g_policyRunner->compute_policy();
                        // Check if policy computation was successful
                        if (policy_result.empty()) {
                            static bool policy_error_logged = false;
                            if (!policy_error_logged) {
                                std::cerr << "Policy computation failed, using safe default positions" << std::endl;
                                policy_error_logged = true;
                            }
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

            //--------------Apply policy actions to robot---------------------
            try {
                if (g_policyRunner) {
                    for(int i = 0; i < 12; i++){
                        // Use scaled_actions if available, otherwise use safe defaults
                        float action_value = g_policyRunner->params.default_joint_angles[i];
                        if (i < static_cast<int>(g_policyRunner->scaled_actions.size())) {
                            // action_value = g_policyRunner->scaled_actions[i];
                        }
                        // std::cout << "Joint " << i << " action_value: " << action_value << " (default: " << g_policyRunner->params.default_joint_angles[i] << ", scaled: " << (i < static_cast<int>(g_policyRunner->scaled_actions.size()) ? g_policyRunner->scaled_actions[i] : 0.0f) << ")" << std::endl;
                        RBQ_API::instance().joint.setPosRef(i, action_value);
                        RBQ_API::instance().joint.setTorqueRef(i, 0);
                        RBQ_API::instance().joint.setGainKpRef(i, g_policyRunner->params.stiffness[i]);
                        RBQ_API::instance().joint.setGainKdRef(i, g_policyRunner->params.damping[i]);
                    }
                    RBQ_API::instance().joint.setAllJointRef();
                }
            } catch (const std::exception& e) {
                static bool action_error_logged = false;
                if (!action_error_logged) {
                    std::cerr << "Action application error: " << e.what() << std::endl;
                    action_error_logged = true;
                }
            } catch (...) {
                static bool action_error_logged = false;
                if (!action_error_logged) {
                    std::cerr << "Unknown action application error" << std::endl;
                    action_error_logged = true;
                }
            }
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

void initializeKeyboard() {
    if (g_keyboardInitialized) return;
    tcgetattr(STDIN_FILENO, &g_oldTio);
    g_newTio = g_oldTio;
    g_newTio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &g_newTio);
    g_keyboardInitialized = true;
}

void cleanupKeyboard() {
    if (g_keyboardInitialized) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_oldTio);
        g_keyboardInitialized = false;
    }
}

char readKeyboard() {
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1) return c;
    return 0;
}

void signalHandler(int signal) {
    std::cout << "Signal received: " << signal << "\n";
    g_isWorking = false;
    cleanupResources();
    std::_Exit(signal);
}

void cleanupResources() {
    std::cout << "Cleaning up...\n";
    g_jointController.reset();
    std::cout << "Resources cleaned.\n";
}

void goToMotionReady() {
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

void goToMotionGround() {
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

void goRLcontrol() {
    if (!g_jointController || !g_policyRunner) return;
    g_jointController->syncReferenceToRobot();
    g_jointController->setAllOwners();

    // Set initial gains - these will be updated by policy
    for(int i=0; i<12; i++){
        RBQ_API::instance().joint.setGainKpRef(i, g_policyRunner->params.stiffness[i]);
        RBQ_API::instance().joint.setGainKdRef(i, g_policyRunner->params.damping[i]);
    }

    std::cout << "RL control initialized with policy gains\n";
}

