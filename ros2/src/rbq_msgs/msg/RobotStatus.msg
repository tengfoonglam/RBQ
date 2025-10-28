
# DOCKING_STATE
int8 DOCKING_MAX_FAIL_CNT_REACHED = -6           # (ABORT SEQENCE) MAX Docking retry count rechead (10 times)
int8 DOCKING_MARKER_POS_INVALID_ROTATION  = -5   # (ABORT SEQENCE) Marker rotation angle is more than +-40 degree
int8 DOCKING_MARKER_POS_INVALID_TOO_FAR   = -4   # (ABORT SEQENCE) Marker is too far more than 5m
int8 DOCKING_MARKER_POS_INVALID_WRONG_DIR = -3   # (ABORT SEQENCE) Marker is detected on front side of robot
int8 DOCKING_MARKER_NOT_FOUND = -2               # (ABORT SEQENCE) Marker not found
int8 DOCKING_FAILED           = -1               # docking failed --> Auto retry
int8 DOCKING_OPERATION_MODE   = 0                # robot is on normal operation
int8 DOCKING_APPROACH_OFFSET  = 1                # 1st reach to charger by offset
int8 DOCKING_APPROACH         = 2                # 2nd reach to charger
int8 DOCKING_APPROACH_WIDE    = 3                # 3rd reach to charger with wide stance
int8 DOCKING_SIT_DOWN         = 4                # sitting down to charger
int8 DOCKING_SUCCESS          = 5                # docking success : charger connected
int8 DOCKING_SUCCESS_CHARGING = 6                # docking success : charging
int8 DOCKING_SUCCESS_NO_CHARGING = 7             # docking success : no_charging


std_msgs/Header header
bool con_start          # Motor control enabled (1 = enabled, 0 = not enabled)
bool ready_pos          # Robot is in ready position (1 = ready, 0 = not ready)
bool ground_pos         # Robot is in ground/sitting position (1 = sitting, 0 = not sitting)
bool force_con          # Force control mode enabled (1 = enabled, 0 = disabled)
bool ext_joy            # External joystick connected (1 = connected, 0 = not connected)
bool is_standing        # Robot is standing (1 = standing, 0 = not standing)
bool can_check          # CAN communication check (1 = success, 0 = failure)
bool find_home          # Encoder homing status (1 = success, 0 = failure)
int8 gait_id            # Current gait mode identifier (returns value defined GAIT_STATE enum)
bool is_fall            # Fall detection status (1 = robot has fallen, 0 = normal)
int8 docking_state      # Docking process status (returns value defined in DOCKING_STATE enum)
bool imu_success        # IMU connection status (1 = success, 0 = failure)

 
