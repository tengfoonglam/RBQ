#include <unistd.h>
#include <QProcess>
#include <QDebug>

#include <QCoreApplication>
#include <QDebug>

// signal to exit infinite loops
extern bool GLOBAL_KILL_SIGNAL;

#include "RobotApiHandler.h"
#include "NetworkHandler.h"
#include <rbq_msgs/msg/high_level_command.hpp>

namespace {
    using Gait = rbq_msgs::msg::HighLevelCommand;
    }    

#include <thread>

NetworkHandler *network = nullptr;
std::thread *th = nullptr;

using namespace RBQ_SDK;

RobotApiHandler::RobotApiHandler(const std::string &host, const int &_commFrequency) : m_ipc()
{
    th = new std::thread(&RobotApiHandler::qAppThread, this, host, _commFrequency);
}

RobotApiHandler::~RobotApiHandler()
{
    QCoreApplication::quit();
}

void RobotApiHandler::qAppThread(const std::string &host, const int &_commFrequency)
{
    int argc=0; char* argv[] = { (char*)"RBQ-client" };
    QCoreApplication a(argc, argv);
    network = new NetworkHandler(&a, host, _commFrequency);
    a.exec();
    network->deleteLater();
    return;
}

Motion::RobotState_t *RobotApiHandler::robotState()
{
    if (network != nullptr) {
        return network->robotState();
    } else {
        return nullptr;
    }
}

Motion::LegStateArray_t *RobotApiHandler::legStateArray()
{
    if (network != nullptr) {
        return network->legStateArray();
    } else {
        return nullptr;
    }
}

Motion::JointStatus_t *RobotApiHandler::jointStatus()
{
    if (network != nullptr) {
        return network->jointStatus();
    } else {
        return nullptr;
    }
}

void RobotApiHandler::setHighLevelCommand(const RBQ_SDK::HighLevelCmd_t &_cmd)
{
    qDebug() << "RobotApiHandler::setHighLevelCommand()";
    if (network != nullptr) {
        QByteArray sendData = QByteArray::fromRawData((const char *) (&_cmd),
                                                      sizeof(HighLevelCmd_t));
        network->sendUDP(sendData);
    }
}

void RobotApiHandler::setJoystickCommand(const sensor_msgs::msg::Joy::SharedPtr joy) const
{
    qDebug() << "RobotApiHandler::setJoystickCommand()";
    if (network != nullptr) {
    
    JoystickCmd_t joystick;
    joystick.axisLeftX = joy->axes[0];
    joystick.axisLeftY = joy->axes[1];
    joystick.axisRightX = joy->axes[2];
    joystick.axisRightY = joy->axes[3];
    joystick.triggerLeft = joy->axes[4];
    joystick.triggerRight = joy->axes[5];
    for(int i=0; i<16; i++) {
        joystick.buttons[i] = joy->buttons[i];
    }
    
    QByteArray sendData = QByteArray::fromRawData((const char *) (&joystick),
                                                      sizeof(JoystickCmd_t));
    network->sendUDP(sendData);
    }

}

void RobotApiHandler::setUserCommand(const USER_COMMAND &usrCmd)
{
    qDebug() << "RobotApiHandler::setUserCommand()";
    if (network != nullptr) {
        QByteArray sendData = QByteArray::fromRawData((const char *) (&usrCmd),
                                                      sizeof(USER_COMMAND));
        network->sendTCP(sendData);
    } else {
        qDebug() << "RobotApiHandler::setUserCommand() failed network is down";
    }
}

void RobotApiHandler::setVisionCommand(RBQ_SDK::GeneralRequest_t &cmd)
{
    if(RBQ_SDK::REQ_ID_PTZ_CALIBRATE <= cmd.requestID && cmd.requestID <= RBQ_SDK::REQ_ID_PTZ_LAST_REQUEST) {
        // qCDebug(NetworkLog) << "setPtzRequest() id: " << _request.requestID;
        m_ipc.setPtzRequest(cmd);
    }
}

void RobotApiHandler::startMotionProgram(const bool &_start, const int &_id)
{
    qDebug() << "RobotApiHandler::startMotionProgram() start:" << _start << "\tid" << _id;
    // APP_NAME_00 = PODOLAN
    // APP_NAME_01 = Daemon
    // APP_NAME_02 = WalkReady
    // APP_NAME_03 = QuadWalk
    // APP_NAME_04 = Manipulator
    USER_COMMAND cmd;

    if(_start) {
        cmd.USER_COMMAND = 1; // start
    } else {
        cmd.USER_COMMAND = 2; // stop
    }
    cmd.USER_PARA_INT[0] = _id;
    cmd.COMMAND_TARGET = FindProgramNumberByName("Motion");

    setUserCommand(cmd);
}

void RobotApiHandler::autoStart()
{
    qDebug() << "RobotApiHandler::autoStart()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("Motion");
    cmd.USER_COMMAND = DAEMON_INIT_AUTOSTART;
    int mode = 1; // Ground pose : 1,
    cmd.USER_PARA_CHAR[1] = mode;
    setUserCommand(cmd);
}

void RobotApiHandler::canCheck()
{
    qDebug() << "RobotApiHandler::canCheck()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("Motion");
    cmd.USER_COMMAND = DAEMON_INIT_CHECK_DEVICE;
    cmd.USER_PARA_CHAR[0] = 12; // ALL
    setUserCommand(cmd);
}

void RobotApiHandler::findHome()
{
    qDebug() << "RobotApiHandler::findHome()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("Motion");
    cmd.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    cmd.USER_PARA_CHAR[0] = -1; // ALL
    int mode = 1; // Ground pose : 1,
    cmd.USER_PARA_CHAR[1] = mode;
    setUserCommand(cmd);
}

void RobotApiHandler::motionStaticLock()
{
    qDebug() << "RobotApiHandler::motionStaticLock()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("WalkReady");
    cmd.USER_COMMAND = WalkReady_CURRENT_POS_LOCK;
    setUserCommand(cmd);
}

void RobotApiHandler::motionStaticReady()
{
    qDebug() << "RobotApiHandler::motionStaticReady()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("WalkReady");
    cmd.USER_COMMAND = WalkReady_MOTION_READY;
    setUserCommand(cmd);
}

void RobotApiHandler::motionStaticGround()
{
    qDebug() << "RobotApiHandler::motionStaticGround()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("WalkReady");
    cmd.USER_COMMAND = WalkReady_MOTION_GROUND;
    setUserCommand(cmd);
}

void RobotApiHandler::motionDynamicGround()
{
    qDebug() << "RobotApiHandler::motionDynamicGround()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_STAND_DOWN;
    setUserCommand(cmd);
    setWalking(false);
}

void RobotApiHandler::motionDynamicReady()
{
    qDebug() << "RobotApiHandler::motionDynamicReady()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_TO_STANCE_MODE;
    setUserCommand(cmd);
    setWalking(false);
}

void RobotApiHandler::motionDynamicWalk()
{
    qDebug() << "RobotApiHandler::motionDynamicWalk()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_TO_WALK_MODE;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionDynamicAim()
{
    qDebug() << "RobotApiHandler::motionDynamicAim()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_TO_AIMING_MODE;
    setUserCommand(cmd);
    setWalking(false);
}

void RobotApiHandler::motionDynamicWalkStairs()
{
    qDebug() << "RobotApiHandler::motionDynamicWalkSlow()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_GAIT_TRANSITION;
    cmd.USER_PARA_CHAR[0] = TO_TROT_S; // Stairs
    setUserCommand(cmd);
    setWalking(true);
}
 

void RobotApiHandler::motionDynamicWalkSlow()
{
    qDebug() << "RobotApiHandler::motionDynamicWalkSlow()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_GAIT_TRANSITION;
    cmd.USER_PARA_CHAR[0] = TO_WAVE; // Wave
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionDynamicRun()
{
    qDebug() << "RobotApiHandler::motionsDynamicRun()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_GAIT_TRANSITION;
    cmd.USER_PARA_CHAR[0] = TO_RUN; // Run
    setUserCommand(cmd);
    setWalking(true);
}

// RL 관련 함수들 구현
void RobotApiHandler::motionRLTrot()
{
    qDebug() << "RobotApiHandler::motionRLTrot()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_TROT;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRLBound()
{
    qDebug() << "RobotApiHandler::motionRLBound()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_BOUND;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRLPace()
{
    qDebug() << "RobotApiHandler::motionRLPace()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_PACE;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRLPronk()
{
    qDebug() << "RobotApiHandler::motionRLPronk()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_PRONK;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRL3LegHR()
{
    qDebug() << "RobotApiHandler::motionRL3LegHR()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_3LEG_HR;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRL3LegHL()
{
    qDebug() << "RobotApiHandler::motionRL3LegHL()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_3LEG_HL;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRL3LegFR()
{
    qDebug() << "RobotApiHandler::motionRL3LegFR()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_3LEG_FR;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRL3LegFL()
{
    qDebug() << "RobotApiHandler::motionRL3LegFL()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_3LEG_FL;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRLTrotVision()
{
    qDebug() << "RobotApiHandler::motionRLTrotVision()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_TROT_VISION;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRLTrotRun()
{
    qDebug() << "RobotApiHandler::motionRLTrotRun()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_TROT_RUN;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRLSilent()
{
    qDebug() << "RobotApiHandler::motionRLSilent()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_SILENT;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRLFrontWalk()
{
    qDebug() << "RobotApiHandler::motionRLFrontWalk()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_FRONT_WALK;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRLHindWalk()
{
    qDebug() << "RobotApiHandler::motionRLHindWalk()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_HIND_WALK;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRLLeftWalk()
{
    qDebug() << "RobotApiHandler::motionRLLeftWalk()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_LEFT_WALK;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRLRightWalk()
{
    qDebug() << "RobotApiHandler::motionRLRightWalk()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_RIGHT_WALK;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionRLStairs()
{
    qDebug() << "RobotApiHandler::motionRLStairs()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_POLICY_CHANGE;
    cmd.USER_PARA_CHAR[0] = RL_STAIRS;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionParametersUpdate()
{
    qDebug() << "RobotApiHandler::motionParametersUpdate()";
    qDebug() << " -- max speed: " << m_maxSpeed
             << " -- body height: " << m_bodyHeight
             << " -- foot height: " << m_footHeight;
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_WALK_PARAMETER_UPDATE;
    cmd.USER_PARA_FLOAT[0]= m_bodyHeight;
    cmd.USER_PARA_FLOAT[1]= m_maxSpeed;
    cmd.USER_PARA_FLOAT[2]= m_footHeight;
    setUserCommand(cmd);
}

void RobotApiHandler::motionDynamicNavigateTo(const float &_targetPosX, const float &_targetPosY, const float &_targetRotZ, const int &_mode)
{
    qDebug() << "RobotApiHandler::motionDynamicNavigateTo()";
    qDebug() << " -- max speed: " << m_maxSpeed
             << " -- body height: " << m_bodyHeight
             << " -- foot height: " << m_footHeight;
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND    = QuadWalk_WAVE_TO_TARGET_POINT;
    cmd.USER_PARA_FLOAT[0]  = _targetPosX;  // target position X (m)
    cmd.USER_PARA_FLOAT[1]  = _targetPosY;  // target position Y (m)
    cmd.USER_PARA_FLOAT[2]  = _targetRotZ;  // target rotation Z (rad)
    cmd.USER_PARA_FLOAT[3]  = 0;  // offset wrt target position X (m)
    cmd.USER_PARA_FLOAT[4]  = 0;  // offset wrt target position Y (m)

    cmd.USER_PARA_INT[0]    = _mode;  // mode

    setUserCommand(cmd);
}

void RobotApiHandler::recovery1()
{
    qDebug() << "RobotApiHandler::recovery1()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("WalkReady");
    cmd.USER_COMMAND = WalkReady_GO_RECOVERY_READY;
    setUserCommand(cmd);
}

void RobotApiHandler::recovery2()
{
    qDebug() << "RobotApiHandler::recovery2()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("WalkReady");
    cmd.USER_COMMAND = WalkReady_FALL_RECOVERY_MOTION;
    setUserCommand(cmd);
}

void RobotApiHandler::calibrateAccelerometer()
{
    qDebug() << "RobotApiHandler::calibrateAccelerometer()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("Motion");
    cmd.USER_COMMAND = DAEMON_ACC_CALIBRATION;
    setUserCommand(cmd);
}

void RobotApiHandler::comEstimationCompensation(const int &stage)
{
    qDebug() << "RobotApiHandler::comEstimationCompensation()";
    if(-1 < stage && stage < 3) {
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
        cmd.USER_COMMAND = QuadWalk_SYS_ID;
        cmd.USER_PARA_INT[0] = stage;
        setUserCommand(cmd);
    } else {
        qDebug() << "RobotApiHandler::comEstimationCompensation() -- wrong stage: " << stage;
    }
}

void RobotApiHandler::eStop()
{
    qDebug() << "RobotApiHandler::eStop()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_E_STOP;
    setUserCommand(cmd);
}

void RobotApiHandler::switchControlMode(const bool &_external)
{
    qDebug() << "RobotApiHandler::switchControlMode() " << _external;
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("Motion");
    cmd.USER_COMMAND = DAEMON_EXT_JOYSTICK_ONOFF;
    cmd.USER_PARA_CHAR[0] = _external;
    setUserCommand(cmd);
}

void RobotApiHandler::powerControl(const PDU_PORT_IDs_e &pdu_port_id, const bool &status)
{
    qDebug() << "RobotApiHandler::powerControl() switch id " << pdu_port_id << " state: " << status;
    USER_COMMAND cmd;
    cmd.USER_COMMAND = DAEMON_PDU_POWER_CONTROL;
    cmd.COMMAND_TARGET = FindProgramNumberByName("Motion");
    cmd.USER_PARA_CHAR[0] = pdu_port_id;
    cmd.USER_PARA_CHAR[1] = status; // 0: OFF;     1: ON;
    setUserCommand(cmd);
}

void RobotApiHandler::switchGait(const int &gait_id)
{
    qDebug() << "RobotApiHandler::switchGait() gait_id:" << gait_id;
    
    switch(gait_id) {
        case Gait::STATE_SIT:
            motionDynamicGround();
            break;
        case Gait::STATE_STAND:
            motionDynamicReady();
            break;
        case Gait::STATE_WALK:
            motionDynamicWalk();
            break;
        case Gait::STATE_AIM:
            motionDynamicAim();
            break;
        case Gait::STATE_STAIRS:
            motionDynamicWalkStairs();
            break;
        case Gait::STATE_WAVE:
            motionDynamicWalkSlow();
            break;
        case Gait::STATE_RUN:
            motionDynamicRun();
            break;
        case Gait::STATE_RL_TROT:
            motionRLTrot();
            break;
        case Gait::STATE_RL_FRONT_WALK:
            motionRLFrontWalk();
            break;
        case Gait::STATE_RL_LEFT_WALK:
            motionRLLeftWalk();
            break;
        case Gait::STATE_RL_RIGHT_WALK:
            motionRLRightWalk();
            break;
        case Gait::STATE_RL_BOUND:
            motionRLBound();
            break;
        case Gait::STATE_RL_PACE:
            motionRLPace();
            break;
        case Gait::STATE_RL_PRONK:
            motionRLPronk();
            break;
        case Gait::STATE_RL_3LEG_HR:
            motionRL3LegHR();
            break;
        case Gait::STATE_RL_3LEG_HL:
            motionRL3LegHL();
            break;
        case Gait::STATE_RL_3LEG_FR:
            motionRL3LegFR();
            break;
        case Gait::STATE_RL_3LEG_FL:
            motionRL3LegFL();
            break;
        case Gait::STATE_RL_TROT_VISION:
            motionRLTrotVision();
            break;
        case Gait::STATE_RL_TROT_RUN:
            motionRLTrotRun();
            break;
        case Gait::STATE_RL_SILENT:
            motionRLSilent();
            break;
        case Gait::STATE_RL_STAIRS:
            motionRLStairs();
            break;
        default:
            qDebug() << "Unknown gait_id:" << gait_id;
            break;
    }
}

void RobotApiHandler::setBodyHeight(const int &newBodyHeight)
{
    qDebug() << "RobotApiHandler::setBodyHeight() :" << newBodyHeight;
    if (-1 < newBodyHeight && newBodyHeight < 101) {
        if (newBodyHeight != m_bodyHeight) {
            m_bodyHeight = newBodyHeight;
            motionParametersUpdate();
        }
    } else {
        qDebug() << "RobotApiHandler::setBodyHeight() invalid height !! ";
    }
}

void RobotApiHandler::setFootHeight(const int &newFootHeight)
{
    qDebug() << "RobotApiHandler::setFootHeight() :" << newFootHeight;
    if (-1 < newFootHeight && newFootHeight < 101) {
        if (newFootHeight != m_footHeight) {
            m_footHeight = newFootHeight;
            motionParametersUpdate();
        }
    } else {
        qDebug() << "RobotApiHandler::setFootHeight() invalid height !! ";
    }
}

void RobotApiHandler::setMaxSpeed(const int &newMaxSpeed)
{
    qDebug() << "RobotApiHandler::setMaxSpeed() :" << newMaxSpeed;
    if (-1 < newMaxSpeed && newMaxSpeed < 101) {
        if (newMaxSpeed != m_maxSpeed) {
            m_maxSpeed = newMaxSpeed;
            motionParametersUpdate();
        }
    } else {
        qDebug() << "RobotApiHandler::setMaxSpeed() invalid speed !! ";
    }
}

void RobotApiHandler::setPanTiltZoom(const float &_pan, const float &_tilt, const float &_zoom)
{
    RBQ_SDK::GeneralRequest_t request_;
    request_.requestID  = RBQ_SDK::REQ_ID_PTZ_PAN_TILT_RAD_ZOOM_X;
    request_.containerFloats[0] = _pan;
    request_.containerFloats[1] = _tilt;
    request_.containerFloats[2] = _zoom;
    setVisionCommand(request_);
}

void RobotApiHandler::docking()
{
    qDebug() << "RobotApiHandler::docking()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = FindProgramNumberByName("QuadWalk");
    cmd.USER_COMMAND = QuadWalk_DOCKING_SEQUENCE_START;
    setUserCommand(cmd);
}

