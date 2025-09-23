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

int main(int argc, char *argv[])
{
    std::cout << "Starting " << APP_NAME << "..." << std::endl;

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

    RBQ_API::instance().initialize(25, false);

    std::cout << "Press q for exit" << std::endl;
    std::cout << "Press w for up" << std::endl;
    std::cout << "Press s for down" << std::endl;
    std::cout << "Press a for left" << std::endl;
    std::cout << "Press d for right" << std::endl;
    std::cout << "Press r for zoom in" << std::endl;
    std::cout << "Press f for zoom out" << std::endl;
    std::cout << "Press c for center" << std::endl;

    float pan = 0.0f;
    float tilt = 0.0f;
    float zoom = 1.0f;

    while(__IS_WORKING)
    {
        char key;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            switch(key) {
            case 'q':
                __IS_WORKING = false;
                break;
            case 'w':
                RBQ_API::instance().ptzCamera.getPanTiltZoom(pan, tilt, zoom);
                tilt -= 0.1f;
                RBQ_API::instance().ptzCamera.setPanTiltZoom(pan, tilt, zoom);
                break;
            case 's':
                RBQ_API::instance().ptzCamera.getPanTiltZoom(pan, tilt, zoom);
                tilt += 0.1f;
                RBQ_API::instance().ptzCamera.setPanTiltZoom(pan, tilt, zoom);
                break;
            case 'a':
                RBQ_API::instance().ptzCamera.getPanTiltZoom(pan, tilt, zoom);
                pan -= 0.1f;
                RBQ_API::instance().ptzCamera.setPanTiltZoom(pan, tilt, zoom);
                break;
            case 'd':
                RBQ_API::instance().ptzCamera.getPanTiltZoom(pan, tilt, zoom);
                pan += 0.1f;
                RBQ_API::instance().ptzCamera.setPanTiltZoom(pan, tilt, zoom);
                break;
            case 'r':
                RBQ_API::instance().ptzCamera.getPanTiltZoom(pan, tilt, zoom);
                zoom += 0.1f;
                RBQ_API::instance().ptzCamera.setPanTiltZoom(pan, tilt, zoom);
                break;
            case 'f':
                RBQ_API::instance().ptzCamera.getPanTiltZoom(pan, tilt, zoom);
                zoom -= 0.1f;
                RBQ_API::instance().ptzCamera.setPanTiltZoom(pan, tilt, zoom);
                break;
            case 'c':
                pan = 0.0f;
                tilt = 0.0f;
                zoom = 1.0f;
                RBQ_API::instance().ptzCamera.setPanTiltZoom(pan, tilt, zoom);
                break;
            default:
                std::cout << "Press q for exit" << std::endl;
                std::cout << "Press w for up" << std::endl;
                std::cout << "Press s for down" << std::endl;
                std::cout << "Press a for left" << std::endl;
                std::cout << "Press d for right" << std::endl;
                std::cout << "Press r for zoom in" << std::endl;
                std::cout << "Press f for zoom out" << std::endl;
                std::cout << "Press c for center" << std::endl;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz
    }
    std::cout << APP_NAME << " is terminating normally" << std::endl;
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
    return 0;
}
