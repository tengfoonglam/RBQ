# gait_state
int8 STATE_FALL                = -2
int8 STATE_OFF                 = -1
int8 STATE_SIT                 = 0
int8 STATE_STAND               = 1
int8 STATE_WALK                = 2
int8 STATE_AIM                 = 3
int8 STATE_STAIRS              = 4
int8 STATE_WAVE                = 5
int8 STATE_RUN                 = 6
int8 STATE_RL_TROT             = 30
int8 STATE_RL_FRONT_WALK       = 31
int8 STATE_RL_LEFT_WALK        = 33
int8 STATE_RL_RIGHT_WALK       = 34
int8 STATE_RL_BOUND            = 35
int8 STATE_RL_PACE             = 36
int8 STATE_RL_PRONK            = 37
int8 STATE_RL_3LEG_HR          = 38
int8 STATE_RL_3LEG_HL          = 39
int8 STATE_RL_3LEG_FR          = 40
int8 STATE_RL_3LEG_FL          = 41
int8 STATE_RL_TROT_VISION      = 42
int8 STATE_RL_TROT_RUN         = 45
int8 STATE_RL_SILENT           = 46
int8 STATE_RL_END              = 50

std_msgs/Header header
string identifier
float64 roll
float64 pitch
float64 yaw
float64 vel_x
float64 vel_y
float64 omega_z
float64 delta_body_h
float64 delta_foot_h
int8	gait_state
bool	gait_transition
