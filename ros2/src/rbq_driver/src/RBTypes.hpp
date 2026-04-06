#ifndef RBTYPES_HPP
#define RBTYPES_HPP

#include <QObject>
#include <QString>
#include <QVector>

#include <array>

#include "half.hpp"

using half_float::half;
using namespace half_float::literal;

namespace RBQ_SDK {

// ------- ID 0~255 ----------
constexpr unsigned char ID_ROBOT_STATE_T                    = 1;
constexpr unsigned char ID_GENERAL_REQUEST                  = 2;
constexpr unsigned char ID_HIGH_LEVEL_COMMAND               = 3;
constexpr unsigned char ID_LEG_STATE_ARRAY                  = 4;

constexpr unsigned char ID_VISION_STATE_T                   = 20;

constexpr unsigned char ID_POINTCLOUD_T                     = 50;

constexpr unsigned char ID_RC_STATE                         = 255;

// ------- ID 0~255 ---------

constexpr unsigned short PORT_ROBOT_MOTION_GAMEPAD_UDP      = 28223;
constexpr unsigned short PORT_ROBOT_MOTION_GAMEPAD_EXT_UDP  = 28224;

constexpr unsigned short PORT_ROBOT_MOTION_UDP              = 56781;
constexpr unsigned short PORT_ROBOT_MOTION_TCP              = 56782;

constexpr unsigned short PORT_ROBOT_VISION_UDP              = 56791;
constexpr unsigned short PORT_ROBOT_VISION_TCP              = 56792;

constexpr uint16_t GCS_UDP_PORT_LIVESTREAM   = 28554;

constexpr static float D2R  = 0.01745329;
constexpr static float R2D  = 57.29577;
constexpr static float PI   = 3.141592;

struct HighLevelCmd_t {
    uint32_t tickWrite  = 0;
    uint32_t tickRead   = 0;

    uint32_t senderIp4Addr = 0;
    unsigned short senderPort = 0;
    bool senderTcp      = false;
    bool accepted       = false;

    float roll              = 0;
    float pitch             = 0;
    float yaw               = 0;
    float vel_x             = 0;
    float vel_y             = 0;
    float omega_z           = 0;
    float delta_body_h      = 0;
    float delta_foot_h      = 0;
    int gaitID              = 0;
    bool gaitTransition     = false;

    /// CommandContainer_t type identifier
    const unsigned char typeIdentifier = 3;

    const unsigned char tail1 =  128;
    const unsigned char tail2 =  127;

    HighLevelCmd_t() {}
    HighLevelCmd_t(const HighLevelCmd_t& p)
    {
        tickWrite       = p.tickWrite;
        tickRead        = p.tickRead;
        senderIp4Addr   = p.senderIp4Addr;
        senderPort      = p.senderPort;
        senderTcp       = p.senderTcp;
        accepted        = p.accepted;

        roll            = p.roll;
        pitch           = p.pitch;
        yaw             = p.yaw;
        vel_x           = p.vel_x;
        vel_y           = p.vel_y;
        omega_z         = p.omega_z;
        delta_body_h    = p.delta_body_h;
        delta_foot_h    = p.delta_foot_h;
        gaitID          = p.gaitID;
        gaitTransition  = p.gaitTransition;
    }
    HighLevelCmd_t& operator=(const HighLevelCmd_t& p)
    {
        tickWrite       = p.tickWrite;
        tickRead        = p.tickRead;
        senderIp4Addr   = p.senderIp4Addr;
        senderPort      = p.senderPort;
        senderTcp       = p.senderTcp;
        accepted        = p.accepted;

        roll            = p.roll;
        pitch           = p.pitch;
        yaw             = p.yaw;
        vel_x           = p.vel_x;
        vel_y           = p.vel_y;
        omega_z         = p.omega_z;
        delta_body_h    = p.delta_body_h;
        delta_foot_h    = p.delta_foot_h;
        gaitID          = p.gaitID;
        gaitTransition  = p.gaitTransition;
        return *this;
    }
};

struct JoystickCmd_t
{
    float axisLeftX = 0.0f;
    float axisLeftY = 0.0f;

    float axisRightX = 0.0f;
    float axisRightY = 0.0f;

    float triggerLeft = 0.0f;
    float triggerRight = 0.0f;

    unsigned char buttons[16] = {0,};
};

struct Request_t {
    const unsigned char head1 =  255;
    const unsigned char head2 =  254;
    /// CommandContainer_t type identifier
    const unsigned char typeIdentifier = ID_GENERAL_REQUEST;

    int requestID = 0;

    uint32_t requestNumber = 0;

    bool requestAccepted = false;

    unsigned char receiverProgramID = 0;

    const static uint8_t containerSize = 10;

    bool            containerBools  [containerSize] = {0,};
    unsigned char   containerUchars [containerSize] = {0,};
    int             containerInts   [containerSize] = {0,};
    float           containerFloats [containerSize] = {0,};

    const unsigned char tail1 =  128;
    const unsigned char tail2 =  127;

    Request_t() {}
    Request_t(const Request_t& p)
    {
        requestID = p.requestID;
        requestNumber = p.requestNumber;
        requestAccepted = p.requestAccepted;
        memcpy(containerBools, p.containerBools, sizeof(bool)*containerSize);
        memcpy(containerUchars, p.containerUchars, sizeof(char)*containerSize);
        memcpy(containerInts, p.containerInts, sizeof(int)*containerSize);
        memcpy(containerFloats, p.containerFloats, sizeof(float)*containerSize);
    }
    Request_t& operator=(const Request_t& p)
    {
        requestID = p.requestID;
        requestNumber = p.requestNumber;
        requestAccepted = p.requestAccepted;
        memcpy(containerBools, p.containerBools, sizeof(bool)*containerSize);
        memcpy(containerUchars, p.containerUchars, sizeof(char)*containerSize);
        memcpy(containerInts, p.containerInts, sizeof(int)*containerSize);
        memcpy(containerFloats, p.containerFloats, sizeof(float)*containerSize);
        return *this;
    }
};

class Motion : public QObject {
    Q_OBJECT
public:

    struct MotorStatus_t {
        unsigned    FET:1;	 	// FET ON
        unsigned    RUN:1;		// Control ON
        unsigned    INIT:1;     // Init Process Passed
        unsigned    MOD:1;		// Control Mode
        unsigned    NON_CTR:1;  // Nonius Count err
        unsigned    BAT:1;      // Low Battery
        unsigned    CALIB:1;    // Calibration Mode
        unsigned    MT_ERR:1;   // Reply Status

        unsigned    JAM:1;		// JAM Error
        unsigned    CUR:1;		// Over Current Error
        unsigned    BIG:1;		// Big Position Error
        unsigned    INP:1;      // Big Input Error
        unsigned    FLT:1;		// FET Driver Fault Error
        unsigned    TMP:1;      // Temperature Error
        unsigned    PS1:1;		// Position Limit Error (Lower)
        unsigned    PS2:1;		// Position Limit Error (Upper)

        unsigned    rsvd:8;

        unsigned    KP:16;
        unsigned    KD:16;
    };

    struct JointState_t {
        bool m_connected      = 0;
        char  temperature   = 0;
        MotorStatus_t status;
        half position       = 0.0_h;
        half torque         = 0.0_h;
    };

    struct BatteryState_t {
        std::array<bool, 2>     charging    = {false,};
        std::array<bool, 2>     m_connected   = {false,};
        std::array<uint8_t, 2>  percentage  = {0,};
        std::array<uint8_t, 2>  voltage     = {0,};
        std::array<uint8_t, 2>  temperature = {0,};
        std::array<half, 2>     current     = {0.0_h,};
    };

    struct PduState_t {
        std::array<bool, 6>     port_05V;
        std::array<bool, 6>     port_12V;
        std::array<bool, 6>     port_52V;
    };

    struct ImuState_t {
        std::array<half, 4> quaternion;    // Quaternion [w,x,y,z]
        std::array<half, 3> rpy;           // Euler angles. Unit is in radians 'rad'
        std::array<half, 3> gyroscope;     // Angular velocity. Unit is in radians per second 'rad/s'
        std::array<half, 3> accelerometer; // Acceleration. Unit is in radians per second squared 'rad/s^2'
        int8_t temperature;                 // Sensor temperature. Unit is in celcius '°C'
    };

    struct GamepadState_t {
        /// Gamepad status. True if m_connected
        bool status = false;
        /// Left Joytick. Float array length of 2
        /// first is vertical 'up-down' axis value. Range: [-1, 1]
        /// second is horizontal 'left-right' axis value. Range: [-1, 1]
        std::array<half, 2> leftJoystick{0.0_h,};
        /// Right Joytick. Float array length of 2
        /// first is vertical 'up-down' axis value. Range: [-1, 1]
        /// second is horizontal 'left-right' axis value. Range: [-1, 1]
        std::array<half, 2> rightJoystick{0.0_h,};

        half leftTrigger = 0.0_h;
        float rightTrigger = 0.0_h;

        /// 20 buttons' state. True if pressed
        std::array<bool, 20>  buttons{false,};

        /// Commander ipv4 address
        uint32_t senderIp4Addr = 0;
    };

    enum StateIdentifier_t : unsigned char {
        undefinedState,
        currentState,
        controlRefState,
        commandRefState,
    };

    struct RobotStatus_t {
        bool    CON_START       = 0;
        bool    READY_POS       = 0;
        bool    GROUND_POS      = 0;
        bool    FORCE_CON       = 0;
        bool    EXT_JOY         = 0;
        bool    IS_STANDING     = 0;
        bool    CAN_CHECK       = 0;
        bool    FIND_HOME       = 0;

        int8_t GAIT_ID          = 0;
        bool    IS_FALL         = 0;
        int8_t  DOCKING_STAT    = 0;
        bool    IMU_SUCCESS     = 0;
        bool    reserve5;
        bool    reserve6;
        bool    reserve7;
        bool    reserve8;

        bool    att00Connected  = 0;    // arm
        bool    att01Connected  = 0;    // att1
        bool    att02Connected  = 0;    // att2
        bool    att03Connected  = 0;    // cctv
        bool    att04Connected  = 0;    // thermal
        bool    att05Connected  = 0;    // ptz
        bool    att06Connected  = 0;
        bool    att07Connected  = 0;

        uint8_t arm_B_0         = 0;
        uint8_t arm_B_1         = 0;
        uint8_t arm_B_2         = 0;
        uint8_t arm_B_3         = 0;
        bool    reserve21;
        bool    reserve22;
        bool    reserve23;
        bool    reserve24;
    };
#ifndef RB_SHARED_MEMORY_H
    struct Velocity_t {
        std::array<half, 3> linear{0.0_h,};
        std::array<half, 3> angular{0.0_h,};
    };

    struct Pose_t {
        std::array<half, 3> position{0.0_h,};
        std::array<half, 3> rpy{0.0_h,};
        std::array<half, 4> quaternion{0.0_h,};
    };

    struct Odometry_t {
        Pose_t pose;
        Velocity_t velocity;
    };

    struct Odometries_t {
        /// Body Velocity wrt Body frame
        Velocity_t velocity;
        /// Body Odometry wrt World frame
        Odometry_t odomWrtWorld;
        /// Body Odometry wrt World frame
        Odometry_t odomWrtMap;
        /// Body Odometry wrt Ground frame
        /// pose.position[2] : body height
        /// pose.rpy : body rotation wrt ground
        /// velocity.angular[2] : omega wrt ground
        /// velocity.linear[0,1] : linear velocity
        Odometry_t odomWrtGround;
    };
#endif
    struct RobotState_t {
        uint32_t tickWrite  = 0;
        uint32_t tickRead   = 0;
        double time = 0;

        uint32_t senderIp4Addr = 0;
        unsigned short senderPort = 0;
        bool senderTcp      = false;
        bool accepted       = false;

        /// Defines Robot's Status
        RobotStatus_t robotStatus;

        /// Battery state
        BatteryState_t batteryState;

        /// Power Delivery Unit state
        PduState_t pduState;

        /// Imu sensor's current state.
        ImuState_t imuState;

        /// Odometries
        Odometries_t odometries;

        /// Active joint count. Max: 20
        uint8_t jointCount = 20;

        /// Joints' state.
        /// 12 [0-11] for legs' joints.
        ///  7 [12-18] for manipulators' joints.
        ///  1 [19] for reserved joint.
        std::array<JointState_t, 20> jointStates;

        /// Accepted remote controller gamepad's current state.
        // GamepadState_t gamepadState;
        /// replacing gamepad with flexible data.
        std::array<char, 40> flexibleData;

        /// Defines if the state is currentState, or referenceState ..
        StateIdentifier_t stateID;

        /// RobotState_t type identifier
        const unsigned char identifier = ID_ROBOT_STATE_T;

        const unsigned char tail1 =  128;
        const unsigned char tail2 =  127;
    };

    enum RequestId_e {
        undefinedRequest                = 0,
        ping                            = 1,
        registerAsNewGcs                = 2,
        sendbackRobotState              = 3,
        REQ_SENDBACK_DEVICES_STATE      = 4,
        REQ_SENDBACK_LEG_STATE_ARRAY    = 5,
        REQ_SENDBACK_JOINT_STATUS       = 104,
    };

    struct RobotPose_t
    {
        float   time        = 0;
        float   x           = 0;
        float   y           = 0;
        float   z           = 0;
        float   rx          = 0;
        float   ry          = 0;
        float   rz          = 0;
        float   fl_leg_0[3] = {0,};
        float   fl_leg_1[3] = {0,};
        float   fl_leg_2[3] = {0,};
        float   fr_leg_0[3] = {0,};
        float   fr_leg_1[3] = {0,};
        float   fr_leg_2[3] = {0,};
        float   rl_leg_0[3] = {0,};
        float   rl_leg_1[3] = {0,};
        float   rl_leg_2[3] = {0,};
        float   rr_leg_0[3] = {0,};
        float   rr_leg_1[3] = {0,};
        float   rr_leg_2[3] = {0,};

        RobotPose_t() { }
        RobotPose_t(const RobotPose_t& p)
        {
            time = p.time;

            x = p.x;
            y = p.y;
            z = p.z;
            rx = p.rx;
            ry = p.ry;
            rz = p.rz;

            memcpy(fl_leg_0, p.fl_leg_0, sizeof(float)*3);
            memcpy(fl_leg_1, p.fl_leg_1, sizeof(float)*3);
            memcpy(fl_leg_2, p.fl_leg_2, sizeof(float)*3);
            memcpy(fr_leg_0, p.fr_leg_0, sizeof(float)*3);
            memcpy(fr_leg_1, p.fr_leg_1, sizeof(float)*3);
            memcpy(fr_leg_2, p.fr_leg_2, sizeof(float)*3);
            memcpy(rl_leg_0, p.rl_leg_0, sizeof(float)*3);
            memcpy(rl_leg_1, p.rl_leg_1, sizeof(float)*3);
            memcpy(rl_leg_2, p.rl_leg_2, sizeof(float)*3);
            memcpy(rr_leg_0, p.rr_leg_0, sizeof(float)*3);
            memcpy(rr_leg_1, p.rr_leg_1, sizeof(float)*3);
            memcpy(rr_leg_2, p.rr_leg_2, sizeof(float)*3);
        }
        RobotPose_t& operator=(const RobotPose_t& p)
        {
            time = p.time;

            x = p.x;
            y = p.y;
            z = p.z;
            rx = p.rx;
            ry = p.ry;
            rz = p.rz;

            memcpy(fl_leg_0, p.fl_leg_0, sizeof(float)*3);
            memcpy(fl_leg_1, p.fl_leg_1, sizeof(float)*3);
            memcpy(fl_leg_2, p.fl_leg_2, sizeof(float)*3);
            memcpy(fr_leg_0, p.fr_leg_0, sizeof(float)*3);
            memcpy(fr_leg_1, p.fr_leg_1, sizeof(float)*3);
            memcpy(fr_leg_2, p.fr_leg_2, sizeof(float)*3);
            memcpy(rl_leg_0, p.rl_leg_0, sizeof(float)*3);
            memcpy(rl_leg_1, p.rl_leg_1, sizeof(float)*3);
            memcpy(rl_leg_2, p.rl_leg_2, sizeof(float)*3);
            memcpy(rr_leg_0, p.rr_leg_0, sizeof(float)*3);
            memcpy(rr_leg_1, p.rr_leg_1, sizeof(float)*3);
            memcpy(rr_leg_2, p.rr_leg_2, sizeof(float)*3);
            return *this;
        }
    };

    struct LegState_t {
        std::array<JointState_t, 3> joint_states;
        std::array<half, 3> foot_target_position_rt_body{0.0_h,};
        std::array<half, 3> foot_position_rt_body{0.0_h,};
        std::array<half, 3> foot_velocity_rt_body{0.0_h,};
        std::array<half, 3> foot_force_vec_rt_body{0.0_h,};
        half foot_force = 0.0_h;
        /// \brief contact
        /// CONTACT_UNKNOWN = 0
        /// CONTACT_MADE = 1
        /// CONTACT_LOST = 2
        uint8_t contact;
    };

    struct LegStateArray_t {
        uint32_t tickWrite  = 0;
        uint32_t tickRead   = 0;

        double time = 0.0;

        std::array<LegState_t, 4> leg_states;

        /// CommandContainer_t type identifier
        const unsigned char typeIdentifier = ID_LEG_STATE_ARRAY;

        const unsigned char tail1 =  128;
        const unsigned char tail2 =  127;
    };

    struct JointStatus_t {
        uint32_t tickWrite = 0;
        uint32_t tickRead = 0;
        double time = 0.0;
        
        struct JointDetail_t {
            bool connected = false;
            char temperature = 0;
            char motor_temp = 0;
            MotorStatus_t status;
            float position_ref = 0.0f;    // Angle Ref
            float position_enc = 0.0f;    // Angle Enc
            float velocity = 0.0f;
            float torque_ref = 0.0f;      // Torque Ref
            float current = 0.0f;         // Current Sen
            float kp = 0.0f;              // KP
            float kd = 0.0f;              // KD
            int owner = 0;                // Owner ID
        };
        
        std::array<JointDetail_t, 16> joint_details;
        const unsigned char typeIdentifier = 6; // REQ_SENDBACK_JOINT_STATUS
        const unsigned char tail1 = 128;
        const unsigned char tail2 = 127;
    };

}; // Motion

typedef std::chrono::high_resolution_clock Clock_t;
typedef std::chrono::high_resolution_clock::time_point TimePoint_t;
typedef std::chrono::duration<double, std::ratio<1> > Seconds_t;

struct GeneralRequest_t
{
    uint32_t tickWrite  = 0;
    uint32_t tickRead   = 0;

    double time = 0;
    TimePoint_t timePointWrite;

    uint32_t senderIp4Addr = 0;
    unsigned short senderPort = 0;
    bool senderTcp = false;

    int requestID = 0;
    bool requestAccepted = false;

    uint8_t containerSize = 10;
    const static uint8_t maxContainerSize = 10;

    bool            containerBools  [maxContainerSize] = {0,};
    unsigned char   containerUchars [maxContainerSize] = {0,};
    int             containerInts   [maxContainerSize] = {0,};
    float           containerFloats [maxContainerSize] = {0,};

    const unsigned char typeIdentifier = ID_GENERAL_REQUEST;
    const unsigned char tail1 =  128;
    const unsigned char tail2 =  127;

    GeneralRequest_t()
    {
        timePointWrite = Clock_t::now();
    }
    GeneralRequest_t(const GeneralRequest_t& p)
    {
        tickWrite           = p.tickWrite;
        tickRead            = p.tickRead;
        time                = p.time;
        timePointWrite      = p.timePointWrite;
        senderIp4Addr       = p.senderIp4Addr;
        senderPort          = p.senderPort;
        senderTcp           = p.senderTcp;
        requestID           = p.requestID;
        requestAccepted     = p.requestAccepted;
        containerSize       = p.containerSize;
        memcpy(containerBools,  p.containerBools,   sizeof(bool)  * std::min(maxContainerSize, p.containerSize));
        memcpy(containerUchars, p.containerUchars,  sizeof(char)  * std::min(maxContainerSize, p.containerSize));
        memcpy(containerInts,   p.containerInts,    sizeof(int)   * std::min(maxContainerSize, p.containerSize));
        memcpy(containerFloats, p.containerFloats,  sizeof(float) * std::min(maxContainerSize, p.containerSize));
    }
    GeneralRequest_t& operator=(const GeneralRequest_t& p)
    {
        tickWrite           = p.tickWrite;
        tickRead            = p.tickRead;
        time                = p.time;
        timePointWrite      = p.timePointWrite;
        requestID           = p.requestID;
        requestAccepted     = p.requestAccepted;
        senderIp4Addr       = p.senderIp4Addr;
        senderPort          = p.senderPort;
        senderTcp           = p.senderTcp;
        memcpy(containerBools, p.containerBools, sizeof(bool)*containerSize);
        memcpy(containerUchars, p.containerUchars, sizeof(char)*containerSize);
        memcpy(containerInts, p.containerInts, sizeof(int)*containerSize);
        memcpy(containerFloats, p.containerFloats, sizeof(float)*containerSize);
        return *this;
    }
    double elapsed() const { return std::chrono::duration_cast<Seconds_t>(Clock_t::now() - timePointWrite).count(); }
};

enum RequestId_e {
    undefinedRequest = 0,

    ping = 1,
    registerAsNewGcs = 2,
    sendbackRobotState = 3,
    sendbackDeviceStates = 4,
    REQ_SENDBACK_LEG_STATE_ARRAY    = 5,

    startProgram = 11,
    killProgram = 12,

    openDevice = 21,
    closeDevice = 22,

    GuideONOFF = 31,
    StartStreamLive = 32,
    DayNightSwitch = 33,

    HeightmapStartCalc = 41,
    HeightmapStateSendback = 42,

    REQ_SENDBACK_PDU_STATE_01       = 101,
    REQ_SENDBACK_DOCK_PARAMS        = 102,
    REQ_SENDBACK_ANIMATION_STATE    = 103,

    // doorHandlePose = 200,
    Handeye_stateSendback   = 199,
    Handeye_doorHandlePose  = 200,
    Handeye_doorPose        = 201,
    Handeye_lastRequest     = 202,


    //SLAM
    Slam_stateSendback  = 299,

    // mapping
    Slam_mapBuild       = 300,
    Slam_mapStop        = 301,
    Slam_mapSave        = 302,
    Slam_mapLoad        = 303,
    Slam_mapReload      = 304,
    Slam_clearObsMap    = 305,

    // localize
    Slam_locStart       = 306,
    Slam_locStop        = 307,
    Slam_locInit        = 308,
    Slam_autoInit       = 309,

    // task
    Slam_taskAdd        = 310,
    Slam_taskDel        = 311,
    Slam_taskSave       = 312,
    Slam_taskLoad       = 313,
    Slam_taskReload     = 314,
    Slam_taskReady      = 315,
    Slam_taskStart      = 316,
    Slam_taskContinue   = 317,
    Slam_taskPause      = 318,
    Slam_taskResume     = 319,
    Slam_taskCancel     = 320,

    // schedule
    Slam_scheduleStart  = 321,
    Slam_scheduleStop   = 322,
    Slam_scheduleLoad   = 323,

    // Annotation
    Slam_annotModeOnOff     = 324,
    Slam_quickAnnotOnOff    = 325,
    Slam_quickAddNode       = 326,
    Slam_annotSave          = 327,
    Slam_clearTopo          = 328,

    // camera view
    Slam_mapColorIntensity   = 329,
    Slam_mapColorHeight      = 330,
    Slam_mapColorNormal      = 331,

    // Slam_modeAnnot  = 332,
    // Slam_modeDrive  = 333,

    Slam_mapPlotKfrm    = 334,
    Slam_mapPlotLive    = 335,

    Slam_pointSize      = 336,

    Slam_plotViewFree   = 337,
    Slam_plotViewTop    = 338,
    Slam_plotViewFollow = 339,

    Slam_streaming      = 340,

    // clip
    Slam_clipOnOff      = 341,
    Slam_clipSize       = 342,

    Slam_lookGoal       = 343,
    Slam_rtb            = 344,
    Slam_eStop          = 345,

    // view
    Slam_view_2d        = 400,
    Slam_view_3d        = 401,
    Slam_view_follow    = 402,

    // settings
    Slam_MAPPING_MODE   = 420,

    // Slam requests here
    Slam_lastRequest            = 999,

    Streamer_stateSendback              = 1000,
    Streamer_videoFrameTouchClicked     = 1001,
    Streamer_videoFrameTouchDragged     = 1002,
    Streamer_videoFrameTouchPressed     = 1003,
    Streamer_videoFrameTouchReleased    = 1004,

    Streamer_keyboardPressed            = 1010,

    Streamer_zoomCCTV                   = 1020,

    Streamer_programStateSendback       = 1050,
    Streamer_deviceStateSendback        = 1051,

    // Streamer requests here
    Streamer_resetVision                = 1998,
    Streamer_lastRequest                = 1999,

    HAL_switchIRProjector               = 2000,
    HAL_lastRequest                     = 2999,

    Aruca_detect                = 3001,
    Aruca_chargerPose           = 3002,
    Aruca_lastRequest           = 3999,



    REQ_ID_UNDEFINED                        = 0,
    REQ_ID_PING                             = 1,
    REQ_ID_REGISTER_AS_NEW_CLIENT           = 2,
    REQ_ID_SENDBACK_ROBOT_STATE             = 3,
    REQ_ID_SENDBACK_DEVICE_STATE            = 4,
    REQ_ID_SENDBACK_LEG_STATE_ARRAY         = 5,
    REQ_ID_SENDBACK_VISION_STATE            = 6,
    REQ_ID_SENDBACK_ADDITIONAL_PTZ_STATE    = 7,

    REQ_ID_SENDBACK_SENSOR_STATES_01       = 10001,
    REQ_ID_SENDBACK_SENSOR_STATES_02       = 10002,

    // ptz related requests start here !!!
    REQ_ID_PTZ_CALIBRATE                    = 12000,
    REQ_ID_PTZ_PAN_TILT_ZOOM_PERCENTAGE     = 12001,
    REQ_ID_PTZ_PAN_TILT_RAD_ZOOM_X          = 12002,
    REQ_ID_PTZ_PAN_TILT_ZOOM_VELOCITY_DEG   = 12006,
    REQ_ID_PTZ_PAN_TILT_ZOOM_STOP           = 12007,
    REQ_ID_PTZ_PAN_TILT_ZOOM_RETURN_CENTER  = 12008,
    REQ_ID_PTZ_ZOOM_X                       = 12009,
    REQ_ID_PTZ_ZOOM_VEL                     = 12010,
    REQ_ID_PTZ_LED_CONTROL                  = 12011,
    REQ_ID_PTZ_IR_LED_CONTROL               = 12012,

    // put the desired request descriptions here !!!
    REQ_ID_PTZ_ROBOT_STATE_SRA_01_SENDBACK  = 12901,

    // ptz related requests end here !!!
    REQ_ID_PTZ_LAST_REQUEST                 = 12999,
};

} // namespace RBQ_SDK

#endif // RBTYPES_HPP

