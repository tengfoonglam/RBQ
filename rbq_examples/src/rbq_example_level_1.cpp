#include <thread>
#include <signal.h>
#include <termios.h>

#include <Eigen/Dense>

#include <rbq_sdk/dds/Subscriber.hpp>
#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/idl/rbq/HighLevelCommand_.hpp>
#include <rbq_sdk/idl/ros2/Joy_.hpp>
// #include <rbq_sdk/idl/rbq/RobotStatus_.hpp>

#include <rbq_sdk/Filter.hpp>

constexpr float kR2D = 57.295779513f;
constexpr float kD2R = 0.0174532925f;

// enum GAIT_STATE : int8_t {
//     FALL_RECOVERY       = -3,
//     FALL_MODE           = -2,
//     CONTROL_OFF         = -1,
//     SITTING             = 0,
//     STANDING            = 1,
//     AIMING              = 2,
//     TROTTING            = 3,
//     TROT_STAIRS         = 4,
//     WAVING              = 5,
//     TROT_RUNNING        = 6,
//     DOOR_OPENING        = 7,
//     ZMP_INITIALIZING    = 8,
//     MANIPULATION        = 9,
//     DOCKING             = 10,
//     DOCKING_SITTING     = 11,
//     RL_TROT             = 30,
//     RL_FRONT_WALK       = 31,
//     RL_HIND_WALK        = 32,
//     RL_LEFT_WALK        = 33,
//     RL_RIGHT_WALK       = 34,
//     RL_BOUND            = 35,
//     RL_PACE             = 36,
//     RL_PRONK            = 37,
//     RL_3LEG_HR          = 38,
//     RL_3LEG_HL          = 39,
//     RL_3LEG_FR          = 40,
//     RL_3LEG_FL          = 41,
//     RL_TROT_VISION      = 42,
//     RL_WHEEL_TROT       = 43,
//     RL_WHEEL_TROT_VISION= 44,
//     RL_TROT_RUN         = 45,
//     RL_SILENT           = 46,
//     RL_STAIRS           = 47,
//     RL_END              = 80,
// };
// const std::map<int, std::string> GAIT_NAMES {
//     {0 , ""},
//     {1 , ""},
//     {2 , ""},
//     {3 , ""},
//     {4 , ""},
//     {5 , ""},
//     {6 , ""},
//     {7 , ""},
//     {8 , ""},
//     {9 , ""},
//     {10, ""},
//     {11, ""},
//     {12, ""},
//     {13, ""},
//     {14, ""},
//     {15, ""},
//     {16, ""},
//     {17, ""},
//     {18, ""},
//     {19, ""},
//     {0, ""},
//     {1, ""},
//     {2, ""},
//     {3, ""},
//     {4, ""},
//     {5, ""},
//     {6, ""},
//     {7, ""},
//     {8, ""},
//     {9, ""},
//     {1, ""},
//     {1, ""},
//     {2, ""},
//     {3, ""},
//     {4, ""},
//     {5, ""},
//     {6, ""},
//     {7, ""},
//     {8, ""},
//     {9, ""},
//     {1, ""},
// };

bool g_isWorking = true;

void signalHandler(int signal) {
    std::cout << "Signal received: " << signal << "\n";
    g_isWorking = false;
    std::_Exit(signal);
}

void printHelp() {
    std::cout << "\n";
    std::cout << " " << APP_NAME << " Help\n";
    std::cout << "\n -p | --path <path> : Specify the policy path to load\n";
    std::cout << "\n";
    std::cout << " Controls:\n";
    std::cout << "  'q' - Quit the application\n";
    std::cout << "  'z' - Motion Sit\n";
    std::cout << "  'x' - Motion Stand\n";
    std::cout << "  'c' - Motion Walk\n";
    std::cout << " Velocity Control (when in control mode):\n";
    std::cout << "  'left joystick up/down' - Forward/Backward\n";
    std::cout << "  'left joystick left/right' - Left/Right\n";
    std::cout << "  'right joystick left/right' - Yaw Left/Right\n\n";
}

int main(int argc, char* argv[])
{
    std::cout << "Starting " << APP_NAME << "...\n";
    printHelp();

    signal(SIGTERM, signalHandler);
    signal(SIGINT,  signalHandler);
    signal(SIGHUP,  signalHandler);
    signal(SIGSEGV, signalHandler);
    signal(SIGKILL, signalHandler);
    signal(SIGSEGV, signalHandler);

    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    // rbq_msgs::msg::dds_::RobotStatus_ msgRobotStatus;
    // rbq_sdk::Subscriber<rbq_msgs::msg::dds_::RobotStatus_>
    //         pubRobotStatus(&msgRobotStatus, "/rt/rbq/info/robotStatus", "lo");

    rbq_msgs::msg::dds_::HighLevelCommand_ msgHighLevelCommand;
    rbq_sdk::Publisher<rbq_msgs::msg::dds_::HighLevelCommand_> pubHighLevelCmd("/rt/rbq/cmd/high_level", "lo");

    while (g_isWorking) {
        char key;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            bool commandUpdated = false;
            switch(key) {
            case 'q':
                g_isWorking = false;
                break;
            case 'z':
                std::cout << "Executing Motion Ground...\n";
                printHelp();
                break;
            case 'x':
                std::cout << "Executing Motion Ready...\n";
                printHelp();
                break;
            case 'c':
                std::cout << "Executing Motion Walk...\n";
                printHelp();
                break;
            case ' ':
                std::cout << "\n Robot Status: \n";
                // std::cout << ":" << msgRobotStatus;
                printHelp();
                break;
            case 'k':
                std::cout << "\n Emergency Kill Robot \n";
                printHelp();
                break;
            case '0':
                std::cout << "\n Initialize Robot \n";
                printHelp();
                break;
            case '1':
                std::cout << "\n Sit \n";
                msgHighLevelCommand.gait_state() = 42;
                printHelp();
                break;
            case '2':
                std::cout << "\n Stand \n";
                msgHighLevelCommand.gait_state() = 42;
                printHelp();
                break;
            case '3':
                std::cout << "\n Blind Walk\n";
                msgHighLevelCommand.gait_state() = 42;
                printHelp();
                break;
            case '4':
                std::cout << "\n Stairs Walk \n";
                msgHighLevelCommand.gait_state() = 42;
                printHelp();
                break;
            case '5':
                std::cout << "\n Blind Walk [RL] \n";
                msgHighLevelCommand.gait_state() = 42;
                printHelp();
                break;
            case '6':
                std::cout << "\n Vision Walk [RL] \n";
                msgHighLevelCommand.gait_state() = 42;
                printHelp();
                break;
            case '7':
                std::cout << "\n Stairs Walk [RL] \n";
                msgHighLevelCommand.gait_state() = 42;
                printHelp();
                break;
            default:
                printHelp();
                break;
            }
            if (commandUpdated) {
                pubHighLevelCmd.write(msgHighLevelCommand);

            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Shutting down...\n";
    return 0;
}
