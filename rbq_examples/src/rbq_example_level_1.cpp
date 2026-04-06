#include <iostream>
#include <thread>
#include <termios.h>
#include <signal.h>

#include "rcl/Api.h"

int __IS_WORKING = true;

void CatchSignals(const int signal) {
    std::cout << "Received signal: " << signal << std::endl;
    switch(signal) {
    case SIGHUP:
    case SIGINT:
    case SIGTERM:
    case SIGKILL:
    case SIGSEGV:
        __IS_WORKING = false;
        std::_Exit(signal);
        break;
    };
}

void printUsage()
{
    std::cout << "\n -- Click:" << std::endl;
    std::cout << "\t 1 --> Sit" << std::endl;
    std::cout << "\t 2 --> Stance" << std::endl;
    std::cout << "\t 3 --> Walk" << std::endl;
    std::cout << "\t 4 --> Stairs" << std::endl;
    std::cout << "\t 5 --> Running" << std::endl;
    std::cout << "\t q --> Quit" << std::endl;
}

int main(int argc, char *argv[])
{
    std::cout << "\n\tStarting " << APP_NAME << "..." << std::endl;
    std::cout << "\n\t" << APP_NAME << " usage:" << std::endl;
    std::cout << "\n\t   ./" << APP_NAME << std::endl;
    std::cout << "\n\tThis example connects to the robot and sends "
                 "\n\t   simple motion commands based on keyboard input." << std::endl;

    signal(SIGTERM, CatchSignals);
    signal(SIGINT,  CatchSignals);
    signal(SIGHUP,  CatchSignals);
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    RBQ_API::instance().initialize(21, true);

    printUsage();

    while(__IS_WORKING)
    {
        char key;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            bool joyCommandUpdated = false;
            switch(key) {
            case 'q':
                __IS_WORKING = false;
                break;
            case '1':
                std::cout << "Sit command received." << std::endl;
                RBQ_API::instance().motion.sit();
                printUsage();
                break;
            case '2':
                std::cout << "Stance command received." << std::endl;
                RBQ_API::instance().motion.stand();
                printUsage();
                break;
            case '3':
                std::cout << "Walk command received." << std::endl;
                RBQ_API::instance().motion.walk();
                printUsage();
                break;
            case '4':
                std::cout << "Stairs command received." << std::endl;
                RBQ_API::instance().motion.stairs();
                printUsage();
                break;
            case '5':
                std::cout << "Running command received." << std::endl;
                RBQ_API::instance().motion.run();
                printUsage();
                break;
                // case 'w':
                //     joyCommandUpdated = true;
                //     rbqApi->gamepad.setLeftJogX(1.0f);
                //     break;
                // case 's':
                //     joyCommandUpdated = true;
                //     rbqApi->gamepad.setLeftJogX(-1.0f);
                //     break;
                // case 'a':
                //     joyCommandUpdated = true;
                //     rbqApi->gamepad.setLeftJogY(-1.0f);
                //     break;
                // case 'd':
                //     joyCommandUpdated = true;
                //     rbqApi->gamepad.setLeftJogY(1.0f);
                //     break;
            }
            if(joyCommandUpdated) {
                // rbqApi->gamepad.sendCommand();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz
    }
    std::cout << APP_NAME << " is terminating normally" << std::endl;
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
    return 0;
}
