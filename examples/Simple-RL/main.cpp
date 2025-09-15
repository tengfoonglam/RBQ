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
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "rcl/Api.h"
#include "rcl/Thread.h"
#include "rcl/Parameters.h"
#include "JointControl.h"
#include "PolicyRunner.h"

constexpr float kR2D = 57.295779513f;
constexpr float kD2R = 0.0174532925f;
constexpr long kControlPeriodUs = kControlPeriodMs * 1000;

JointTable g_jointTable;
std::unique_ptr<RBQ_API> g_api;
std::unique_ptr<JointController> g_jointController;
std::unique_ptr<PolicyRunner> g_policyRunner;


int g_jointJogNo = 0;
int g_jogMoveDir = 0;
bool g_isWorking = false;

struct termios g_oldTio, g_newTio;
bool g_keyboardInitialized = false;

// Add declaration for initial joint positions
float initial_joint_pos[12] = {0,};

enum class TaskState {
    Idle = 0,
    Motion,
    JogMove,
    PolicyControl,
};
TaskState g_currentTask = TaskState::Idle;

enum class UserCommand {
    None = 0,
    Test,
    MotionReady,
    MotionGround,
    RLcontrol,
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
    std::cout << "Press 'x' for MOTION_GROUND, 'z' for MOTION_READY, 'c' for RLcontrol, 'q' to quit.\n";

    initializeKeyboard();

    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);
    signal(SIGHUP, signalHandler);
    signal(SIGSEGV, signalHandler);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "Memory lock failed.\n";
        return 1;
    }

    Parameters params;
    if (params.Initialize("hw_configs/QuadParameter.ini")) {
        std::cerr << "Failed to initialize parameters.\n";
        return 1;
    }

    try {
        g_policyRunner = std::make_unique<PolicyRunner>();
        g_api = std::make_unique<RBQ_API>(20);
        g_jointController = std::make_unique<JointController>(g_api.get(), kMaxJoint);
        g_jointController->syncReferenceToRobot();
    
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

    while (g_isWorking && g_api && g_jointController && g_policyRunner) {
        char key = readKeyboard();
        UserCommand command = UserCommand::None;

        switch (key) {
            case 'x': command = UserCommand::MotionGround; break;
            case 'z': command = UserCommand::MotionReady; break;
            case 'c': command = UserCommand::RLcontrol; break;
            case 'q': g_isWorking = false; break;
        }

        switch (command) {
            case UserCommand::MotionReady:
                std::cout << "Executing Motion Ready...\n";
                g_currentTask = TaskState::Motion;
                goToMotionReady();
                g_currentTask = TaskState::Idle;
                break;

            case UserCommand::MotionGround:
                std::cout << "Executing Motion Ground...\n";
                g_currentTask = TaskState::Motion;
                goToMotionGround();
                g_currentTask = TaskState::Idle;
                break;
            
            case UserCommand::RLcontrol:
                std::cout << "Executing RLcontrol...\n";             
                g_currentTask = TaskState::PolicyControl;
                std::cout << "checkout 1 " << std::endl;
                for (int i = 0; i < 12; i++) {
                    initial_joint_pos[i] = static_cast<float>(g_jointController->getAngle(i)); 
                }
                g_policyRunner->initialize_observation(initial_joint_pos);
                std::cout << "checkout 2 " << std::endl;
                goRLcontrol();

                break;

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

    while (g_isWorking && g_api && g_jointController && g_policyRunner) {

        switch (g_currentTask) {
            case TaskState::Idle:
                break;

            case TaskState::Motion:
                g_jointController->updateAllJoints();
                g_jointController->sendReferencesToRobot();
                break;

            case TaskState::PolicyControl:
            {
                // Ensure motion ownership is maintained
                static bool ownership_set = false;
                if (!ownership_set) {
                    std::cout << "Setting motion ownership in control loop..." << std::endl;
                    for (int i = 0; i < 12; i++) {
                        g_api->joint.setMotionOwner(i);
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
                std::cout << "checkout 3 " << std::endl;
                for(int i=0; i<12; i++){
                    g_api->joint.getPos(i, current_joint_pos[i]);
                    g_api->joint.getVel(i, current_joint_vel[i]);
                    g_api->joint.getTorque(i, current_joint_torque[i]);
                    g_api->joint.getGainKp(i, current_joint_kp[i]);
                    g_api->joint.getGainKd(i, current_joint_kd[i]);
                }

                //-------------imu--------------
                Eigen::Quaternion<float> quat;
                Eigen::Vector<float, 3> gyro;
                Eigen::Vector<float, 3> acc;
                Eigen::Vector<float, 3> rpy;
                std::cout << "checkout 4 " << std::endl;

                g_api->imu.getQuaternion(quat);
                g_api->imu.getGyro(gyro);
                g_api->imu.getAcc(acc);
                g_api->imu.getRPY(rpy);

                //-------------gamepad----------
                // Initialize with safe default values
                float left_jog_x = 0.0f;
                float left_jog_y = 0.0f;
                float left_trigger = 0.0f;
                float right_jog_x = 0.0f;
                float right_jog_y = 0.0f;
                float right_trigger = 0.0f;

                bool BTN_A = false, BTN_B = false, BTN_X = false, BTN_Y = false;
                bool BTN_LB = false, BTN_RB = false;
                bool BTN_BACK = false, BTN_START = false, BTN_LOGI = false;
                bool BTN_LJOG = false, BTN_RJOG = false;
                bool ARROW_UP = false, ARROW_DW = false, ARROW_L = false, ARROW_R = false;
                std::cout << "checkout 5 " << std::endl;
                // Safely access gamepad - skip if not available
                bool gamepad_available = false;
                try {
                    if (g_api) {
                        g_api->gamepad.getLeftJogX(left_jog_x);
                        g_api->gamepad.getLeftJogY(left_jog_y);
                        g_api->gamepad.getLeftTrigger(left_trigger);
                        g_api->gamepad.getRightJogX(right_jog_x);
                        g_api->gamepad.getRightJogY(right_jog_y);
                        g_api->gamepad.getRightTrigger(right_trigger);

                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::A, BTN_A);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::B, BTN_B);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::X, BTN_X);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::Y, BTN_Y);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::LB, BTN_LB);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::RB, BTN_RB);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::BACK, BTN_BACK);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::START, BTN_START);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::LOGITECH, BTN_LOGI);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::LJOG, BTN_LJOG);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::RJOG, BTN_RJOG);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_UP, ARROW_UP);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_DOWN, ARROW_DW);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_LEFT, ARROW_L);
                        g_api->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_RIGHT, ARROW_R);
                        
                        gamepad_available = true;
                    }
                } catch (const std::exception& e) {
                    static bool gamepad_error_logged = false;
                    if (!gamepad_error_logged) {
                        std::cerr << "Gamepad access error (using defaults): " << e.what() << std::endl;
                        gamepad_error_logged = true;
                    }
                    // Keep default values (all zeros/false)
                } catch (...) {
                    static bool gamepad_error_logged = false;
                    if (!gamepad_error_logged) {
                        std::cerr << "Unknown gamepad access error (using defaults)" << std::endl;
                        gamepad_error_logged = true;
                    }
                }
                std::cout << "checkout 6 " << std::endl;

                //--------------50Hz policy run-------------------------------------
                static int decimation_cnt = 0;
                if(decimation_cnt == 10){
                    decimation_cnt = 0;
                    std::cout << "checkout 7 " << std::endl;
                    try {
                        if (g_policyRunner) {
                            std::cout << "checkout 8 " << std::endl;
                            g_policyRunner->set_base_ang_vel(gyro);
                            g_policyRunner->set_projected_gravity(quat);
                            g_policyRunner->set_joint_pos(current_joint_pos);
                            g_policyRunner->set_joint_vel(current_joint_vel);
                            std::cout << "checkout 9 " << std::endl;
                            // Use gamepad commands if available, otherwise use zero commands
                            if (gamepad_available) {
                                g_policyRunner->set_commands(0, 0, 0);  //TODO : check gamepad command
                            } else {
                                g_policyRunner->set_commands(0.0f, 0.0f, 0.0f);
                            }
                            std::cout << "checkout 10 " << std::endl;
                            auto policy_result = g_policyRunner->compute_policy();
                            std::cout << "checkout 11 " << std::endl;
                            // Check if policy computation was successful
                            if (policy_result.empty()) {
                                static bool policy_error_logged = false;
                                if (!policy_error_logged) {
                                    std::cerr << "Policy computation failed, using safe default positions" << std::endl;
                                    policy_error_logged = true;
                                }
                            }
                            std::cout << "checkout 12 " << std::endl;
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
                std::cout << "checkout 13 " << std::endl;
                decimation_cnt++;

                //--------------Apply policy actions to robot---------------------
                try {
                    if (g_policyRunner && g_api) {
                        for(int i = 0; i < 12; i++){
                            // Use scaled_actions if available, otherwise use safe defaults
                            float action_value = g_policyRunner->params.default_joint_angles[i];
                            if (i < static_cast<int>(g_policyRunner->scaled_actions.size())) {
                                action_value = g_policyRunner->scaled_actions[i];
                            }
                            std::cout << "checkout 14 " << std::endl;
                            std::cout << "Joint " << i << " action_value: " << action_value << " (default: " << g_policyRunner->params.default_joint_angles[i] << ", scaled: " << (i < static_cast<int>(g_policyRunner->scaled_actions.size()) ? g_policyRunner->scaled_actions[i] : 0.0f) << ")" << std::endl;
                            g_api->joint.setPosRef(i, action_value);
                            g_api->joint.setTorqueRef(i, 0);
                            g_api->joint.setGainKp(i, g_policyRunner->params.stiffness[i]);
                            g_api->joint.setGainKd(i, g_policyRunner->params.damping[i]);
                            std::cout << "checkout 15 " << std::endl;
                        }
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
    g_policyRunner.reset();
    g_jointController.reset();
    g_api.reset();
    std::cout << "Resources cleaned.\n";
}

void goToMotionReady() {
    if (!g_jointController || !g_api) return;
    g_jointController->syncReferenceToRobot();
    g_jointController->setAllOwners();

    for (int i = 0; i < kMaxJoint; ++i) {
        g_api->joint.setGainKp(i, 200.0f);
        g_api->joint.setGainKd(i, 2.5f);
    }
    usleep(500 * 1000);

    float motionTime = 1400.0f;
    float pitchAngles[4];
    g_api->joint.getPosRef(RBQ_API::Joint::JointID::HRP, pitchAngles[0]);
    g_api->joint.getPosRef(RBQ_API::Joint::JointID::HLP, pitchAngles[1]);
    g_api->joint.getPosRef(RBQ_API::Joint::JointID::FRP, pitchAngles[2]);
    g_api->joint.getPosRef(RBQ_API::Joint::JointID::FLP, pitchAngles[3]);

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
    if (!g_jointController || !g_api) return;
    g_jointController->syncReferenceToRobot();
    g_jointController->setAllOwners();

    for (int i = 0; i < kMaxJoint; ++i) {
        g_api->joint.setGainKp(i, 200.0f);
        g_api->joint.setGainKd(i, 2.5f);
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
    if (!g_jointController || !g_api || !g_policyRunner) return;
    g_jointController->syncReferenceToRobot();
    g_jointController->setAllOwners();

    // Set initial gains - these will be updated by policy
    for(int i=0; i<12; i++){
        g_api->joint.setGainKp(i, g_policyRunner->params.stiffness[i]);
        g_api->joint.setGainKd(i, g_policyRunner->params.damping[i]);
    }
    
    std::cout << "RL control initialized with policy gains\n";
}
