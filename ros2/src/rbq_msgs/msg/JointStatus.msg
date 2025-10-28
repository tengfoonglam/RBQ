std_msgs/Header header

# Joint Detail Information
# Connection and Temperature Status
bool[] connected          # Joint connection status (true = connected, false = disconnected)
int8[] temperature        # Board temperature in Celsius
int8[] motor_temp         # Motor temperature in Celsius

# Motor Status Flags
bool[] status_fet         # FET (Field Effect Transistor) status (true = ON, false = OFF)
bool[] status_run         # Motor running status (true = running, false = stopped)
bool[] status_init        # Initialization status (true = initialized, false = not initialized)
bool[] status_mod         # Control mode status (true = position control, false = torque control)
bool[] status_non_ctr     # Nonius count error (true = error, false = normal)
bool[] status_bat         # Battery status (true = low battery, false = normal)
bool[] status_calib       # Calibration status (true = in calibration mode, false = normal)
bool[] status_mt_err      # Motor error status (true = error, false = normal)
bool[] status_jam         # JAM error status (true = jammed, false = normal)
bool[] status_cur         # Over current error (true = over current, false = normal)
bool[] status_big         # Big position error (true = large error, false = normal)
bool[] status_inp         # Input error (true = input error, false = normal)
bool[] status_flt         # FET driver fault error (true = fault, false = normal)
bool[] status_tmp         # Temperature error (true = over temperature, false = normal)
bool[] status_ps1         # Position limit error - lower bound (true = limit reached, false = normal)
bool[] status_ps2         # Position limit error - upper bound (true = limit reached, false = normal)
bool[] status_rsvd        # Reserved status bit

# Control Data
float32[] position_ref    # Reference position in radians
float32[] position_enc    # Encoder position in radians
float32[] velocity        # Joint velocity in rad/s
float32[] torque_ref      # Reference torque in Nm
float32[] current         # Motor current in Amperes
float32[] kp              # Proportional gain (P gain)
float32[] kd              # Derivative gain (D gain)
int32[] owner             # Joint owner ID (which controller owns this joint)