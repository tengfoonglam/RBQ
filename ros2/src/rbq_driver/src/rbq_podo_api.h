#ifndef RBQ_PODO_API_H
#define RBQ_PODO_API_H

#include <string>
#include <map>

const std::map<int, std::string> APP_NAMES {
    //-------------------------------------Internal Programs----------------------------------
    {0 , "Motion"},
    {1 , "Network"},
    {2 , "WalkReady"},
    {3 , "QuadWalk"},
    {4 , "ManiControl"},
    {5 , "Calibration"},
    {6 , "Estimation"},
    {7 , "RcEquipment"},
    {8 , "RLWalk"},
    {9 , "GUI"},
    {10, "0"},
    {11, "0"},
    {12, "0"},
    {13, "0"},
    {14, "0"},
    {15, "0"},
    {16, "0"},
    {17, "0"},
    {18, "0"},
    {19, "0"},
    //---------------------------------------------------------------------------------------

    //----------------------------------External Programs for User---------------------------
    {20, "0"},
    {21, "0"},
    {22, "0"},
    {23, "0"},
    {24, "0"},
    {25, "0"},
    {26, "0"},
    {27, "0"},
    {28, "0"},
    {29, "0"},
    {30, "0"},
    {31, "0"},
    {32, "0"},
    {33, "0"},
    {34, "0"},
    {35, "0"},
    {36, "0"},
    {37, "0"},
    {38, "0"},
    {39, "0"},
    //---------------------------------------------------------------------------------------
};

static inline int FindProgramNumberByName(const std::string& _name) {
    for (const auto& pair : APP_NAMES) {
        if (pair.second == _name) {
            return pair.first;
        }
    }
    return -1;
}

// GUI to ROBOT
struct USER_COMMAND
{
    int     COMMAND_TARGET          = 0;
    int     USER_COMMAND            = 0;
    char    USER_PARA_CHAR  [40]    = {0,};
    int     USER_PARA_INT   [40]    = {0,};
    float   USER_PARA_FLOAT [40]    = {0,};
    double  USER_PARA_DOUBLE[40]    = {0,};
};

// ---------Daemon AL------------
enum _DAEMON_COMMAND_SET_{
    NO_ACT = 0,
    // For process handle
    DAEMON_PROCESS_CREATE,
    DAEMON_PROCESS_KILL,
    // For initialize
    DAEMON_INIT_CHECK_DEVICE = 100,
    DAEMON_INIT_FIND_HOME,
    DAEMON_INIT_FET_ONOFF,
    DAEMON_INIT_CONTROL_ONOFF,
    DAEMON_INIT_REF_ONOFF,
    DAEMON_INIT_MOTOR_ONOFF,
    DAEMON_TEST_TORQUE,
    DAEMON_INIT_AUTOSTART,
    // For Setting
    DAEMON_SETTING_HOME_ZERO = 200,
    DAEMON_SETTING_REQ_FOC_GAIN,
    DAEMON_SETTING_SET_FOC_GAIN,
    DAEMON_SETTING_DQ_ALIGN,
    DAEMON_SETTING_CUR_NULLING,
    // For sensor
    DAEMON_SENSOR_ENCODER_RESET = 300,
    DAEMON_SENSOR_ENCODER_ONOFF,
    DAEMON_SENSOR_SENSOR_ONOFF,
    DAEMON_SENSOR_FT_NULL,
    DAEMON_SENSOR_IMU_NULL,
    DAEMON_SENSOR_IMU_OFFSET_SET,
    DAEMON_SENSOR_FOG_NULL,
    DAEMON_SENSOR_FOG_USB_RESET,
    DAEMON_SENSOR_OF_NULL,
    DAEMON_SENSOR_OF_LAMP_ONOFF,
    // For motion
    DAEMON_MOTION_REF_ONOFF = 400,
    DAEMON_MOTION_MOVE,
    DAEMON_MOTION_GAIN_OVERRIDE,
    DAEMON_MOTION_ERROR_CLEAR,
    // For CAN
    DAEMON_CAN_ENABLE_DISABLE,
    //
    POWER_CONTROL,
    DAEMON_TFB_TX_MSG,

    //new
    DAEMON_SET_DGAIN_MODE,
    DAEMON_SETTING_MOTOR_RECOVERY,
    DAEMON_SETTING_PITCH_MULTITURN_RESET,
    DAEMON_ACC_CALIBRATION,
    DAEMON_SET_CURRENT_LIMIT,
    DAEMON_GET_CURRENT_LIMIT,

    // PDU
    DAEMON_SOFT_EMERGENCY = 500,
    DAEMON_12V_48V_ONOFF,       // DAEMON_VISION_PC
    DAEMON_USB_BOARD,

    DAEMON_SET_ENCODER_OFFSET, // uiuk added

    DAEMON_EXT_JOYSTICK_ONOFF                       = 400,

    DAEMON_LAN2CAN_GENERAL_MSG                      = 600,
    DAEMON_SERIAL_MSG                               = 601,
    DAEMON_LAN2CAN_SOCKET_CLOSE_AND_REOPEN          = 602,
    DAEMON_PDU_POWER_CONTROL                        = 603,
    DAEMON_PDU_DEVICE_CONTROL                       = 604,
    DAEMON_PDU_DEVICE_SETTING                       = 605,

    DAEMON_VISION_AL                                = 700,

    /*! Manipulator */
    DAEMON_MANIPULATOR_INIT = 800,
    DAEMON_MANIPULATOR_REF_ON,
    DAEMON_MANI_AUTO_READY,
    DAEMON_MANIPULATOR_ERROR_CLEAR, // to be implemented
    DAEMON_MANIPULATOR_RECOVERY, // to be implemented

    DAEMON_E_STOP                                   = 1000,
    DAEMON_QUADWALK_AL_RESTART                      = 1001,
};

//-------- QuadWalk AL--------
enum _QuadWalk_COMMAND_SET_
{
    QuadWalk_NO_ACT                 = 100,
    QuadWalk_MPC_PARA_UPDATE 		= 101,
    QuadWalk_MPC_WALK_READY         = 102,
    QuadWalk_SWING_LEG_TEST 		= 103,
    QuadWalk_GAIT_TRANSITION 		= 104,
    QuadWalk_STAND_UP               = 105,
    QuadWalk_STAND_DOWN             = 106,
    QuadWalk_SAVE_DATA_START		= 107,
    QuadWalk_SAVE_DATA_STOP 		= 108,
    QuadWalk_MPC_STAND_UP           = 109,
    QuadWalk_WALK_PARAMETER_UPDATE 	= 110,
    QuadWalk_TO_WALK_MODE           = 111,
    QuadWalk_TO_STANCE_MODE 		= 112,
    QuadWalk_E_STOP                 = 113,
    QuadWalk_TO_AIMING_MODE 		= 114,
    QuadWalk_SYS_ID                 = 115,
    QuadWalk_TO_WALK_SLOW           = 116,
    QuadWalk_TO_HEALTH_CHECK_MODE   = 117,

    QuadWalk_TO_DOOR_OPENING_MODE   = 119,
    QuadWalk_CRUISE_VEL_SET         = 120,
    QuadWalk_VISION_TROT_MODE       = 123,
    QuadWalk_POLICY_RUN             = 124,
    QuadWalk_POLICY_CHANGE          = 125,

    QuadWalk_WAVE_TO_TARGET_POINT   = 200,
    QuadWalk_DOCKING_SIT_DOWN       = 201,
    QuadWalk_DOCKING_SEQUENCE_START = 202,
    QuadWalk_DOCKING_RETRY          = 203,

    QuadWalk_LOW_BATTERY_SEQUENCE   = 300,

    QuadWalk_SAVE_MASS_CALBRATION_PARA = 500,
    QuadWalk_SAVE_ZMP_CALIBRATION_PARA = 501,

};

//-------- WalkReady AL--------
enum _WalkReady_COMMAND_SET_
{
    WalkReady_NO_ACT = 100,
    WalkReady_MOTION_READY,
    WalkReady_MOTION_GROUND,
    WalkReady_MOTION_HOMMING,
    WalkReady_MOTION_LIFT,
    WalkReady_MOTION_STAND_UP,
    WalkReady_GO_RECOVERY_READY,
    WalkReady_FALL_RECOVERY_MOTION,
    WalkReady_CURRENT_POS_LOCK,
    WalkReady_JOINT_LOCK_UNLOCK,
    WalkReady_JOINT_SPACE_JOG,
    WalkReady_FLIPOVER_PREVENTION,
};

enum _WALKING_GAIT_TANSITION{
    NONE = 0,
    STANCE_TO_WALK,
    WALK_TO_STANCE,
    TO_STANCE,
    TO_WAVE,
    TO_TROT,
    TO_TROT_FAST,
    TO_TROT_SLOW,
    TO_PACE,
    TO_RUN,
    TO_PRONK,
    TO_BOUND,
    WAVE_READY,
    STANCE_READY,
    TO_TROT_V,
    TO_TROT_S, //stairs
    TO_TROT_Q, //Quiet
};

enum GAIT_STATE : int8_t {
    FALL_RECOVERY       = -3,
    FALL_MODE           = -2,
    CONTROL_OFF         = -1,
    SITTING             = 0,
    STANDING            = 1,
    AIMING              = 2,
    TROTTING            = 3,
    TROT_STAIRS         = 4,
    WAVING              = 5,
    TROT_RUNNING        = 6,
    DOOR_OPENING        = 7,
    ZMP_INITIALIZING    = 8,
    MANIPULATION        = 9,
    DOCKING             = 10,
    DOCKING_SITTING     = 11,

    //RL_STATE
    RL_TROT             = 30,
    RL_FRONT_WALK       = 31,
    RL_HIND_WALK        = 32,
    RL_LEFT_WALK        = 33,
    RL_RIGHT_WALK       = 34,
    RL_BOUND            = 35,
    RL_PACE             = 36,
    RL_PRONK            = 37,
    RL_3LEG_HR          = 38,
    RL_3LEG_HL          = 39,
    RL_3LEG_FR          = 40,
    RL_3LEG_FL          = 41,
    RL_TROT_VISION      = 42,
    RL_WHEEL_TROT       = 43,
    RL_WHEEL_TROT_VISION= 44,
    RL_TROT_RUN         = 45,
    RL_SILENT           = 46,
    RL_STAIRS           = 47,
    RL_END              = 50,
};

// ------ PDU ----------------
enum PDU_PORT_IDs_e : unsigned char {
    PDU_PORT_48V_LEG                    = 0x00,
    PDU_PORT_48V_ADD                    = 0x01,
    PDU_PORT_48V_EXT                    = 0x02,

    PDU_PORT_12V_VisionPC               = 0x10,
    PDU_PORT_12V_COMM                   = 0x11,
    PDU_PORT_12V_Lidar                  = 0x12,
    PDU_PORT_12V_CCTV                   = 0x13,
    PDU_PORT_12V_THER                   = 0x14,
    PDU_PORT_12V_IRLed                  = 0x15,
    PDU_PORT_12V_Speaker                = 0x16,

    PDU_PORT_5V_CAMERAS                 = 0x20,
    PDU_PORT_5V_AUDIO_SIDE_CAM_USBHUB   = 0x21,
};


#endif // RBQ_PODO_API_H
