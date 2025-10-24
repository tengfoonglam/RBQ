#ifndef RBQ_API_H
#define RBQ_API_H

#include <mutex>

#if defined(PRIVATE)
#include "SharedMemory.h"
#endif
#include <Eigen/Dense>

class StateEstimator;

class RBQ_API {
public:
    static RBQ_API& instance();
    RBQ_API(const RBQ_API&) = delete;
    RBQ_API& operator=(const RBQ_API&) = delete;
    RBQ_API(RBQ_API&&) = delete;
    RBQ_API& operator=(RBQ_API&&) = delete;

    /**
     * @brief initialize an RBQ_API instance with a specific process ID.
     *
     * This function assigns a unique process ID to this control instance.
     * Process IDs in the range [20, 39] are reserved for user-level applications.
     * This allows multiple user processes or threads to interact with the robot independently.
     *
     * Since multiple processes can attempt to control the robot simultaneously,
     * each joint uses a **Motion Owner** mechanism to ensure safety and consistency.
     * Only the process that owns a joint can control it.
     * To take control of a joint, call `setMotionOwner()` before sending any reference commands.
     *
     * @param _processId The ID of this process (should be in range [20, 39] for user applications).
     */
    int initialize(const int &_processId, const bool _local); //for user id range = [20, 39]
#if defined(PRIVATE)
    int initialize(const bool _local);
#endif

    bool isInitialized() { return m_initialized; }

    struct Motion {
        /**
         * @defgroup MotionAPI High-Level Motion Command API
         * @brief API for high-level motion control
         *
         * This group provides commands to control high-level locomotion behaviors such as
         * walking, posture transitions, and interaction motions.
         *
         * These functions are designed to be used sequentially in accordance with the robot's
         * internal motion progress and state transitions. It is **not recommended** to call
         * multiple commands in rapid succession without confirming the completion of the previous one.
         * Improper sequencing may result in command conflicts or incomplete motions.
         */

        Motion(RBQ_API* parent) : m_parent(parent) {}

        void startProgram(const bool &_start, const int &_id);

        /**
         * @ingroup MotionAPI
         * @brief Executes full system initialization and activates all joint controllers.
         *
         * Initiates the startup sequence after the robot is manually placed in the predefined
         * **Initialize Pose** (refer to the *Operation Guide* for posture details).
         *
         * The sequence includes:
         * - **CAN Check**: Communication verification with all motor controllers.
         * - **Find Home**: Encoder initialization for all joints.
         * - **Control Start**: Motor activation and control loop engagement.
         *
         * Completion status for each step is indicated via `StatusWord`.
         */
        void autoStart();

        /**
         * @ingroup MotionAPI
         * @brief Transitions the robot to the standing posture using joint position control.
         *
         * Moves the robot from the current pose to a predefined standing posture
         * by commanding joint positions directly (position control mode).
         *
         * This transition is static and does not involve dynamic balancing or feedback control.
         */
        void staticStand();

        /**
         * @ingroup MotionAPI
         * @brief Transitions the robot to the sitting posture using joint position control.
         *
         * Commands the robot to move from the current pose to a predefined sitting posture
         * by directly setting joint positions (position control mode).
         *
         * This is a static transition without dynamic balance control.
         */
        void staticSit();

        /**
         * @ingroup MotionAPI
         * @brief Transitions to a sitting posture using dynamic control.
         */
        void sit();

        /**
         * @ingroup MotionAPI
         * @brief Transitions to a standing posture using dynamic control.
         *
         * After execution, high-level posture control is enabled via the HighLevelControl API,
         * allowing adjustment of roll, pitch, yaw, and body height.
         */
        void stand();

        /**
         * @ingroup MotionAPI
         * @brief Switches to walking mode with dynamic locomotion control.
         *
         * Enables forward/backward, lateral, and rotational walking via the HighLevelControl API.
         * Use parametersUpdate() to adjust body height, maximum speed, and foot lift height.
         *
         * @see parametersUpdate()
         */
        void walk();

        /**
         * @ingroup MotionAPI
         * @brief Enables stair locomotion mode.
         *
         * Activates stair-walking locomotion. Speed control is available via the HighLevelControl API.
         */
        void stairs();

        void wave();

        /**
         * @ingroup MotionAPI
         * @brief Enables running locomotion mode.
         *
         * The robot moves using dynamic running motions.
         * Speed can be adjusted via the HighLevelControl API.
         */
        void run();

        /**
         * @ingroup MotionAPI
         * @brief Enables posture control in standing mode.
         *
         * Activates a standing posture control mode. Body posture can be adjusted via the HighLevelControl API.
         */
        void aim();

        /**
         * @ingroup MotionAPI
         * @brief Initiates automatic docking sequence near the charging station.
         *
         * Starts an automatic docking sequence. The robot must be positioned so that
         * its rear camera can detect the charging station's marker.
         */
        void docking();

        /**
         * @ingroup MotionAPI
         * @brief Immediately stops the robot by disabling control and applying high damping to all joints.
         *
         * All joints enter a high damping mode and control inputs are released,
         * effectively halting the robot's movement.
         *
         * To recover from this state, call @ref stand(), which will reinitialize the posture
         * and bring the robot back to a standing position.
         */
        void eStop();

        /**
         * @ingroup MotionAPI
         * @brief Immediately reboots the robot pc.
         *
         * To safely execute the command, first, call @ref sit() to sit down, then call @ref eStop() to stop.
         *
         */
        void pcReboot();

        void cruiseControl(const bool &_increase = true);

        /**
         * @ingroup MotionAPI
         * @brief Updates walking parameters such as body height, speed, and foot lift height.
         *
         * Used during walk mode to set:
         * - Body height (`bodyHeight`)
         * - Maximum walking speed (`speed`)
         * - Foot lifting height during swing phase (`footHeight`)
         */
        void parametersUpdate(const uint8_t &bodyHeight, const uint8_t &speed, const uint8_t &footHeight);

        void zmpCalibrate();

        void lockAllJoint();

        int getHighLevelCMD(
            float &roll, float &pitch, float &yaw,
            float &vel_x, float &vel_y, float &omega_z,
            float &delta_body_h, float &delta_foot_h, float &delta_max_speed,
            int &gait_id, bool &gaitTransition);

        int setHighLevelCMD(
            const float &roll,
            const float &pitch,
            const float &yaw,
            const float &vel_x,
            const float &vel_y,
            const float &omega_z,
            const float &delta_body_h,
            const float &delta_foot_h,
            const float &delta_max_speed,
            const int   &gait_id,
            const bool  &gaitTransition);

        int setModeGamepadCommand();

        int setModeHighLevelCommand();

        void HighLevelCmd();

    private:
        RBQ_API* m_parent = nullptr;  // RBQ_API class pointer
    };
    Motion motion{this};

    struct PowerControl {
        /**
         * @defgroup PowerControlAPI Power Control API
         * @brief API for managing the robot's internal and external power ports.
         *
         * The `PowerControl` class provides an interface to control power to various components
         * of the robot including joint actuators, onboard computers, external ports, and peripheral devices.
         *
         * Power ports are categorized by voltage level (48V, 12V, 5V) and function.
         * Each port can be turned ON or OFF programmatically.
         */

        /**
         * @ingroup PowerControlAPI
         * @brief Enumeration of power port list.
         */
        enum class Port : unsigned char {
            Leg_48V     = 0x00,  ///< 48V power for leg actuators (internal)
            AddOn_48V   = 0x01,  ///< 48V Add-on device port (top side)
            Ext_48V     = 0x02,  ///< 48V External power port (top side)

            UserPc_12V     = 0x10,  ///< 12V power for user PC (internal)
            LANPort1_12V   = 0x11,  ///< 12V communication port (top side)
            LANPort2_12V   = 0x12,  ///< 12V LD port (top side)
            LANPort3_12V   = 0x13,  ///< 12V SC port (top side)
            LANPort4_12V   = 0x14,  ///< 12V TC port (top side)
            Panel_12V      = 0x15,  ///< 12V port front/rear panel (front/real side)

            AMP_12V     = 0x16,  ///< 12V speaker amplifier power (internal)
            CAM_5V      = 0x20,  ///< 5V USB power for camera (internal)
            Audio_5V    = 0x21   ///< 5V USB power for audio (internal)
        };

        PowerControl(RBQ_API* parent) : m_parent(parent) {}

        /**
         * @ingroup PowerControlAPI
         * @brief Sets power state of a port by port number.
         *
         * @param _port_num Hardware ID of the port (see Port enum values).
         * @param _on_off Power state: true for ON, false for OFF.
         * @return Returns 1 if command was sent successfully.
         */
        int setPower(const unsigned char _port_num, const bool _on_off);

        /**
         * @ingroup PowerControlAPI
         * @brief Sets power state of a port by enum name.
         *
         * This is a convenience overload using `Port` enum for clarity.
         *
         * @param _port_name Port enum value.
         * @param _on_off Power state: true for ON, false for OFF.
         * @return Returns 1 if command was sent successfully.
         * 
         * Example:
         * @code
         * // Turn off 48V power to the leg actuators
         * api->powerControl.setPower(RBQ_API::PowerControl::Port::Leg_48V, 0);
         * @endcode
         *
         * @see setPower(const unsigned char, const bool)
         */
        int setPower(const Port _port_name, const bool _on_off){
            return setPower(static_cast<int>(_port_name), _on_off);
        }

        /**
         * @ingroup PowerControlAPI
         * @brief Gets power state using Port enum.
         *
         * This overload allows checking power status using the Port enum for better readability.
         *
         * @param _port_name The port to check (as Port enum).
         * @param _on_off Reference to receive the current power state (true = on, false = off).
         * @return Result of the power state query.
         *
         * Example:
         * @code
         * // check 48V power on/off status of leg actuators
         * bool on_off;
         * api->powerControl.getPowerState(RBQ_API::PowerControl::Port::Leg_48V, on_off);
         * @endcode
         *
         * @see getPowerState(const unsigned char, bool&)
         */
        int getPowerState(const Port _port_name, bool &_on_off);

        /**
         * @ingroup PowerControlAPI
         * @brief Retrieves the current battery voltage.
         *
         * @param out_battery_voltage_ Reference to store the measured battery voltage (in volts).
         * @return Returns 1 if the voltage was successfully retrieved.
         */
        int getBatteryVoltage(float &out_battery_voltage_);

    private:
        RBQ_API* m_parent = nullptr;  // RBQ_API class pointer

        void pdu_power_control(unsigned char _index, unsigned char _onoff);
    };
    PowerControl powerControl{this};

    struct Status {
        /**
         * @defgroup StatusAPI Status API
         * @brief API for querying robot status information.
         *
         * This class provides access to various internal robot status flags such as CAN communication,
         * homing completion, control start state, gait mode, docking state, and fall detection.
         *
         * Each status can be queried individually via the getStatusWord() function.
         */

        Status(RBQ_API* parent) : m_parent(parent) {}

        /**
         * @ingroup StatusAPI
         * @brief Robot status identifiers for querying system state.
         */
        enum class STAT {
            CAN_CHECK,       ///< CAN communication check (1 = success, 0 = failure)
            FIND_HOME,       ///< Encoder homing status (1 = success, 0 = failure)
            CON_START,       ///< Motor control enabled (1 = enabled, 0 = not enabled)
            GAIT_ID,         ///< Current gait mode identifier (returns value defined GAIT_STATE enum)
            DOCKING_STATE,   ///< Docking process status (returns value defined in DOCKING_STATE enum)
            IS_FALL,         ///< Fall detection status (1 = robot has fallen, 0 = normal)
            IMU_SUCCESS,     ///< IMU connection status (1 = success, 0 = failure)
        };

        /**
         * @ingroup StatusAPI
         * @brief Enumeration of gait state identifiers.
         *
         * These values are used as return values for the GAIT_ID status query.
         */
        enum GAIT_STATE : int8_t {
            FALL_RECOVERY       = -3, ///< value = -3
            FALL_MODE           = -2, ///< value = -2
            CONTROL_OFF         = -1, ///< value = -1
            SITTING             = 0,  ///< value = 0
            STANDING            = 1,  ///< value = 1
            AIMING              = 2,  ///< value = 2
            TROTTING            = 3,  ///< value = 3
            TROT_STAIRS         = 4,  ///< value = 4
            WAVING              = 5,  ///< value = 5
            TROT_RUNNING        = 6,  ///< value = 6
            DOOR_OPENING        = 7,  ///< value = 7
            ZMP_INITIALIZING    = 8,  ///< value = 8
            MANIPULATION        = 9,  ///< value = 9
            DOCKING             = 10, ///< value = 10
            DOCKING_SITTING     = 11, ///< value = 11

            //RL_STATE
            RL_TROT             = 30, ///< value = 30
            RL_FRONT_WALK       = 31, ///< value = 31
            RL_HIND_WALK        = 32, ///< value = 32
            RL_LEFT_WALK        = 33, ///< value = 33
            RL_RIGHT_WALK       = 34, ///< value = 34
            RL_BOUND            = 35, ///< value = 35
            RL_PACE             = 36, ///< value = 36
            RL_PRONK            = 37, ///< value = 37
            RL_3LEG_HR          = 38, ///< value = 38
            RL_3LEG_HL          = 39, ///< value = 39
            RL_3LEG_FR          = 40, ///< value = 40
            RL_3LEG_FL          = 41, ///< value = 41
            RL_TROT_VISION      = 42, ///< value = 42
            RL_END              = 50, ///< value = 50
        };

        /**
         * @ingroup StatusAPI
         * @brief Enumeration of docking process status.
         *
         * This enum defines the possible stages and error states in the robot's docking procedure.
         * These values are used as return values for the DOCKING_STATE status query.
         */
        enum DOCKING_STATE : int8_t {
            DOCKING_MAX_FAIL_CNT_REACHED        = -6, ///< Max retry count (10) reached. Docking aborted.
            DOCKING_MARKER_POS_INVALID_ROTATION = -5, ///< Marker rotation angle > ±40°. Docking aborted.
            DOCKING_MARKER_POS_INVALID_TOO_FAR  = -4, ///< Marker too far (> 5m). Docking aborted.
            DOCKING_MARKER_POS_INVALID_WRONG_DIR= -3, ///< Marker detected on wrong side. Docking aborted.
            DOCKING_MARKER_NOT_FOUND            = -2, ///< Marker not found. Docking aborted.
            DOCKING_FAILED                      = -1, ///< Docking failed. Retry will be attempted.

            DOCKING_OPERATION_MODE              = 0,  ///< Robot is in normal operation mode (not docking).
            DOCKING_APPROACH_OFFSET             = 1,  ///< First offset approach toward charger.
            DOCKING_APPROACH                    = 2,  ///< Second direct approach toward charger.
            DOCKING_APPROACH_WIDE               = 3,  ///< Third approach with wide stance.
            DOCKING_SIT_DOWN                    = 4,  ///< Sitting down to connect to charger.
            DOCKING_SUCCESS                     = 5,  ///< Docking successful, charger physically connected.
            DOCKING_SUCCESS_CHARGING            = 6,  ///< Docking successful, and charging is active.
            DOCKING_SUCCESS_NO_CHARGING         = 7,  ///< Docking successful, but not charging.
        };

        /**
         * @brief Retrieves the specified status flag.
         *
         * @param _stat The desired status type to query.
         * @param status_ returned status value.
         *
         * @return Result of the getStatusWord query success = 1.
         *
         * Example:
         * @code
         * // Check if leg motor controller enabled
         * int control_status;
         * api->status.getStatusWord(RBQ_API::Status::STAT::CON_START, control_status);
         * @endcode
         * 
         * @ingroup StatusAPI
         */
        int getStatusWord(const STAT &_stat, int &status_);

    private:
        RBQ_API* m_parent = nullptr;  // RBQ_API class pointer
    };
    Status status{this};

    struct Joint {
        /**
         * @defgroup JointControlAPI Joint Control API
         * @brief APIs for controlling joint-level commands such as position, torque, and gains.
         *
         * This group includes functions to set joint position references, torque references,
         * and control gains (Kp and Kd) for each joint. These APIs are designed to support
         * real-time joint control in a multi-process system architecture.
         *
         * ### Motion Owner
         * Although there is only one physical robot, multiple processes may exist that generate reference commands.
         * In order for a specific process to control a particular joint, it must first acquire motion ownership
         * of that joint. If a process sets references without owning the joint, those references will not be applied
         * to the robot. To ensure control, call setMotionOwner(const int &_jointId) before using any reference-setting functions
         * (e.g., position, torque, gain).
         *
         * ### Damping Gain Modes
         * - **Dgain Mode 1**:
         *   vel_ref is generated by differentiating pos_ref within the low-level motor controller.
         *   The applied joint torque is:
         *   torque = kp * (pos_ref - pos_meas) + kd * (vel_ref - vel_meas) + torque_feedforward
         *
         * - **Dgain Mode 2**:
         *   vel_ref is fixed to zero.
         *   The applied joint torque is:
         *   torque = kp * (pos_ref - pos_meas) + kd * (-vel_meas) + torque_feedforward
         */

        Joint(RBQ_API* parent) : m_parent(parent) {}

        /**
         * @ingroup JointControlAPI
         * @brief Enumeration of all joint IDs in the quadruped robot.
         *
         * The robot has 4 legs: hind-left, hind-right, front-left, and front-right.
         * Each leg has 3 joints: roll (R), pitch (P), and knee (K).
         * This enum provides symbolic names for each joint, corresponding to their index.
         */
        enum class JointID : int {
            HRR = 0,  ///< Hind-right roll joint
            HRP = 1,  ///< Hind-right pitch joint
            HRK = 2,  ///< Hind-right knee joint

            HLR = 3,  ///< Hind-left roll joint
            HLP = 4,  ///< Hind-left pitch joint
            HLK = 5,  ///< Hind-left knee joint

            FRR = 6,  ///< Front-right roll joint
            FRP = 7,  ///< Front-right pitch joint
            FRK = 8,  ///< Front-right knee joint

            FLR = 9,  ///< Front-left roll joint
            FLP = 10, ///< Front-left pitch joint
            FLK = 11, ///< Front-left knee joint

            HRW = 12, ///< Hind-right wheel (RBQ-W)
            HLW = 13, ///< Hind-left wheel (RBQ-W)
            FRW = 14, ///< Front-right wheel (RBQ-W)
            FLW = 15, ///< Front-left wheel (RBQ-W)
        };


        // ───────────────────────────────────────────────────────────────
        // Joint Torque Limits by Joint Type [Nm]
        // ───────────────────────────────────────────────────────────────

        static constexpr float TORQUE_ROLL_UPPER  = 104.f;   ///< @brief Upper bound torque for roll joints [Nm] @ingroup JointControlAPI
        static constexpr float TORQUE_ROLL_LOWER  = -104.f;  ///< @brief Lower bound torque for roll joints [Nm] @ingroup JointControlAPI

        static constexpr float TORQUE_PITCH_UPPER = 104.f;   ///< @brief Upper bound torque for pitch joints [Nm] @ingroup JointControlAPI
        static constexpr float TORQUE_PITCH_LOWER = -104.f;  ///< @brief Lower bound torque for pitch joints [Nm] @ingroup JointControlAPI

        static constexpr float TORQUE_KNEE_UPPER  = 140.f;   ///< @brief Upper bound torque for knee joints [Nm] @ingroup JointControlAPI
        static constexpr float TORQUE_KNEE_LOWER  = -70.f;   ///< @brief Lower bound torque for knee joints [Nm] @ingroup JointControlAPI

        /**
         * @brief Set all joint references from local to the robot.
         *
         * This function synchronizes the all joint reference values (position, torque, Kp, Kd)
         * from the the local variable within this API instance to the robot.
         * It should be called periodically once after setting each joint reference values.
         * Such as at the end of the each control loop iteration.
         *
         * Example:
         * @code
         * for (int i=0; i<12; i++) {
         *     RBQ_API::instance().joint.setPosRef(i, desired_position[i]);
         *     RBQ_API::instance().joint.setTorqueRef(i, desired_torque[i]);
         *     RBQ_API::instance().joint.setGainKpRef(i, desired_kp[i]);
         *     RBQ_API::instance().joint.setGainKdRef(i, desired_kd[i]);
         * }
         * RBQ_API::instance().joint.setAllJointRef();
         * @endcode
         *
         * @return Returns 1 on successful update.
         *
         * @ingroup JointControlAPI
         */
        int setAllJointRef();

        /**
         * @brief Assigns motion ownership of the specified joint to the current process.
         *
         * In a multi-process control architecture, multiple processes may generate reference commands.
         * However, only the process that owns a joint (the motion owner) can actively control it.
         *
         * This function assigns the current process as the motion owner of the specified joint.
         * When ownership is assigned, the existing reference values (position, torque, Kp, Kd)
         * are initialized using the values from the default system process.
         *
         * This function must be called once before using any reference-setting functions
         * (e.g., position, torque, gain). Once motion ownership is acquired, it remains valid
         * until another process explicitly takes ownership of the same joint.
         * If this process is not the motion owner, any reference it sets will be ignored by the controller.
         *
         * @param _jointId The joint ID to take ownership of (valid range: 0 to 11).
         *
         * @return Returns 1 if ownership is successfully assigned;
         *         -2 if _jointId is out of range.
         *
         * @ingroup JointControlAPI
         */
        int setMotionOwner(const int &_jointId);

        /**
         * @brief Overload using JointID.
         * @see setMotionOwner(const int&)
         * @ingroup JointControlAPI
         */
        int setMotionOwner(const JointID _jointId){
            return setMotionOwner(static_cast<int>(_jointId));
        }

        /**
         * @brief Sets the joint position reference.
         *
         * This function sets the desired joint position (in radians) for the specified joint.
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param _joint_pos_rad The desired joint position in radians.
         *
         * @return Returns 1 if the position is successfully set and the current process is the motion owner;
         *         -1 if the position is set but the current process is not the motion owner;
         *         -2 if _jointId is out of range.
         *
         * @note This function requires motion ownership. Use setMotionOwner() before calling.
         * @ingroup JointControlAPI
         */
        int setPosRef(const int &_jointId, const float &_joint_pos_rad);

        /**
         * @brief Overload using JointID.
         * @see setPosRef(const int&, const float&)
         * @ingroup JointControlAPI
         */
        int setPosRef(const JointID _jointId, const float &_joint_pos_rad){
            return setPosRef(static_cast<int>(_jointId), _joint_pos_rad);
        }

        /**
         * @brief Sets the joint torque reference in Newton-meters (Nm).
         *
         * This function sets the desired torque value for a specific joint.
         * The input torque is clamped to a valid range based on the joint type:
         * - Roll joints:   [TORQUE_ROLL_LOWER, TORQUE_ROLL_UPPER]
         * - Pitch joints:  [TORQUE_PITCH_LOWER, TORQUE_PITCH_UPPER]
         * - Knee joints:   [TORQUE_KNEE_LOWER, TORQUE_KNEE_UPPER]
         *
         * If the input value is outside the allowed range, it is clamped to the nearest valid value.
         * To ensure the reference takes effect, the calling process must first acquire motion ownership
         * using `setMotionOwner()`.
         *
         * @param _jointId The ID of the joint (valid range: 0 to 11, wheel: 12 to 15).
         * @param _torque_Nm The desired torque in Newton-meters (Nm).
         *
         * @return Returns:
         * -  1 if the torque is set and the current process is the motion owner.
         * -  2 if the torque is set but the input value was clamped.
         * - -1 if the torque is set but the current process is not the motion owner.
         * - -2 if _jointId is out of range.
         *
         * @note This function requires motion ownership. Use setMotionOwner() before calling.
         * @ingroup JointControlAPI
         */
        int setTorqueRef(const int &_jointId, const float &_torque_Nm);

        /**
         * @brief Overload using JointID.
         * @see setTorqueRef(const int&, const float&)
         * @ingroup JointControlAPI
         */
        int setTorqueRef(const JointID _jointId, const float &_torque_Nm){
            return setTorqueRef(static_cast<int>(_jointId), _torque_Nm);
        }

        /**
         * @brief Sets the joint position gain (Kp) with quantization.
         *
         * This function sets the desired Kp gain for a specified joint after quantizing it based on predefined resolution steps.
         * For roll/pitch joints (IDs: 0, 1, 3, 4, 6, 7, 9, 10), the quantization step is 17.62734 Nm/rad, and for knee joints (IDs: 2, 5, 8, 11),
         * the quantization step is 25.55443 Nm/rad. The quantized gain is applied and the actual gain (nearest multiple of the resolution step)
         * is returned via the real_kp_ parameter.
         *
         * @param _jointId The ID of the joint (valid range: 0 to 11, wheel: 12 to 15).
         * @param _set_kp_ The desired Kp gain value in Nm/rad (valid range: 0.0 to 5000.0).
         * @param real_kp_ Reference parameter to receive the actual applied gain value after quantization.
         *
         * @return  1 if the gain is set and the current process is the motion owner;
         *         -1 if the gain is set but the current process is not the motion owner;
         *         -2 if _jointId is out of range;
         *         -3 if _set_kp_ is out of range.
         *
         * @note This function requires motion ownership. Use setMotionOwner() before calling.
         *
         * @ingroup JointControlAPI
         */
        int setGainKpRef(const int &_jointId, const float &_set_kp, float &real_kp_);

        /**
         * @brief Overload using JointID.
         * @see setGainKpRef(const int&, const float&, float&)
         * @ingroup JointControlAPI
         */
        int setGainKpRef(const JointID _jointId, const float &_set_kp, float &real_kp_){
            return setGainKpRef(static_cast<int>(_jointId), _set_kp, real_kp_);
        }

        /**
         * @brief Overload without return gain value.
         * @see setGainKpRef(const int&, const float&, float&)
         * @ingroup JointControlAPI
         */
        int setGainKpRef(const int &_jointId, const float &_set_kp){
            float dummy;
            return setGainKpRef(_jointId, _set_kp, dummy);
        }

        /**
         * @brief Overload using JointID, and without return gain value.
         * @see setGainKpRef(const int&, const float&, float&)
         * @ingroup JointControlAPI
         */
        int setGainKpRef(const JointID _jointId, const float &_set_kp){
            float dummy;
            return setGainKpRef(static_cast<int>(_jointId), _set_kp, dummy);
        }

        /**
         * @brief Sets the joint damping gain (Kd) with quantization.
         *
         * This function sets the desired Kd gain for a specified joint after quantizing it based on predefined resolution steps.
         * For roll/pitch joints (IDs: 0, 1, 3, 4, 6, 7, 9, 10), the quantization step is 0.035255 Nm*s/rad, and for knee joints (IDs: 2, 5, 8, 11),
         * the quantization step is 0.051109 Nm*s/rad. The quantized gain is applied and the actual gain (i.e. the nearest multiple of the resolution step)
         * is returned via the real_kd_ parameter.
         *
         * @param _jointId The ID of the joint (valid range: 0 to 11, wheel: 12 to 15).
         * @param _set_kd The desired Kd gain value in Nm*s/rad (valid range: 0.0 to 10.0).
         * @param real_kd_ Reference parameter to receive the actual applied damping gain value after quantization.
         *
         * @return Returns  1 if the gain is set and the current process is the motion owner;
         *         Returns -1 if the gain is set but the current process is not the motion owner;
         *         Returns -2 if _jointId is out of range;
         *         Returns -3 if _set_kd is out of range.
         *
         * @note This function requires motion ownership. Use setMotionOwner() before calling.
         *
         * @ingroup JointControlAPI
         */
        int setGainKdRef(const int &_jointId, const float &_set_kd, float &real_kd_);

        /**
         * @brief Overload using JointID.
         * @see setGainKdRef(const int&, const float&, float&)
         * @ingroup JointControlAPI
         */
        int setGainKdRef(const JointID _jointId, const float &_set_kd, float &real_kd_){
            return setGainKdRef(static_cast<int>(_jointId), _set_kd, real_kd_);
        }

        /**
         * @brief Overload without return gain value.
         * @see setGainKdRef(const int&, const float&, float&)
         * @ingroup JointControlAPI
         */
        int setGainKdRef(const int &_jointId, const float &_set_kd) {
            float dummy;
            return setGainKdRef(_jointId, _set_kd, dummy);
        }

        /**
         * @brief Overload using JointID, without return gain value.
         * @see setGainKdRef(const int&, const float&, float&)
         * @ingroup JointControlAPI
         */
        int setGainKdRef(const JointID _jointId, const float &_set_kd) {
            float dummy;
            return setGainKdRef(static_cast<int>(_jointId), _set_kd, dummy);
        }

        /**
         * @brief Returns the current joint position in radians.
         *
         * This function retrieves the measured joint angle for the specified joint.
         * The value is obtained from the sensor and expressed in radians.
         *
         * @param _jointId The ID of the joint (valid range: 0 to 11, wheel: 12 to 15).
         * @param joint_pos_rad_ Reference variable that will be filled with the measured joint position (in radians).
         *
         * @return 1 : success, -1 : fail to access data, -2 : joint ID is out of range.
         *
         * @ingroup JointControlAPI
         */
        int getPos(const int &_jointId, float &joint_pos_rad_);

        /**
         * @brief Overload using JointID.
         * @see getPos(const int&, float&)
         * @ingroup JointControlAPI
         */
        int getPos(const JointID _jointId, float &joint_pos_rad_){
            return getPos(static_cast<int>(_jointId), joint_pos_rad_);
        }

        /**
         * @brief Returns the current joint velocity in radians per second.
         *
         * This function retrieves the measured joint angular velocity for the specified joint.
         * The value is obtained from the sensor and expressed in radians per second (rad/s).
         *
         * @param _jointId The ID of the joint (valid range: 0 to 11, wheel: 12 to 15).
         * @param joint_vel_rad_per_sec_ Reference variable that will be filled with the measured joint velocity (in rad/s).
         *
         * @return 1 : success, -1 : fail to access data, -2 : joint ID is out of range.
         *
         * @ingroup JointControlAPI
         */
        int getVel(const int &_jointId, float &joint_vel_rad_per_sec_);

        /**
         * @brief Overload using JointID.
         * @see getVel(const int&, float&)
         * @ingroup JointControlAPI
         */
        int getVel(const JointID _jointId, float &joint_vel_rad_per_sec_){
            return getVel(static_cast<int>(_jointId), joint_vel_rad_per_sec_);
        }

        /**
         * @brief Returns the current joint torque in Newton-meters (Nm).
         *
         * This function retrieves the measured torque value applied to the specified joint.
         * The value is obtained from the sensor and expressed in Newton-meters.
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param joint_torque_ Reference variable that will be filled with the measured joint torque (in Nm).
         *
         * @return 1 : success, -1 : fail to access data, -2 : joint ID is out of range.
         * @ingroup JointControlAPI
         */
        int getTorque(const int &_jointId, float &joint_torque_);

        /**
         * @brief Overload using JointID.
         * @see getTorque(const int&, float&)
         * @ingroup JointControlAPI
         */
        int getTorque(const JointID _jointId, float &joint_torque_){
            return getTorque(static_cast<int>(_jointId), joint_torque_);
        }

        /**
         * @brief Retrieves the joint position gain (Kp).
         *
         * This function calculates and returns the current joint position gain value in Nm/rad for the specified joint.
         * It applies a resolution of 17.62734 Nm/rad for roll/pitch (IDs: 0, 1, 3, 4, 6, 7, 9, 10) joints and 25.55443 Nm/rad for knee joints (IDs: 2, 5, 8, 11).
         *
         * @param joint_kp_ Reference parameter to receive the gain value in Nm/rad.
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         *
         * @return 1 : success, -1 : fail to access data, -2 : joint ID is out of range.
         * @ingroup JointControlAPI
         */
        int getGainKp(const int &_jointId, float &joint_kp_);

        /**
         * @brief Overload using JointID.
         * @see getGainKp(const int&, float&)
         * @ingroup JointControlAPI
         */
        int getGainKp(const JointID _jointId, float &joint_kp_){
            return getGainKp(static_cast<int>(_jointId), joint_kp_);
        }

        /**
         * @brief Retrieves the joint damping gain (Kd).
         *
         * This function calculates and returns the current joint damping gain value in Nm*s/rad for the specified joint.
         * It uses a resolution of 0.035255 Nm*s/rad for roll/pitch joints (IDs: 0, 1, 3, 4, 6, 7, 9, 10) and 0.051109 Nm*s/rad for knee joints (IDs: 2, 5, 8, 11).
         *
         * @param joint_kd_ Reference parameter to receive the damping gain value in Nm*s/rad.
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         *
         * @return 1 : success, -1 : fail to access data, -2 : joint ID is out of range.
         * @ingroup JointControlAPI
         */
        int getGainKd(const int &_jointId, float &joint_kd_);

        /**
         * @brief Overload using JointID.
         * @see getGainKd(const int&, int&)
         * @ingroup JointControlAPI
         */
        int getGainKd(const JointID _jointId, float &joint_kd_){
            return getGainKd(static_cast<int>(_jointId), joint_kd_);
        }

        /**
         * @brief Returns the current position reference of the specified joint.
         *
         * Retrieves the position reference (in radians) set for the joint by the system process.
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param joint_pos_ref_ Reference to store the position reference (in radians).
         * @return 1 : success, -1 : fail to access data, -2 : joint ID is out of range.
         * @ingroup JointControlAPI
         */
        int getPosRef(const int &_jointId, float &joint_pos_ref_);

        /**
         * @brief Overload using JointID.
         * @see getPosRef(const int&, float&)
         * @ingroup JointControlAPI
         */
        int getPosRef(const JointID _jointId, float &joint_pos_rad_){
            return getPosRef(static_cast<int>(_jointId), joint_pos_rad_);
        }

        /**
         * @brief Returns the current torque reference of the specified joint.
         *
         * Retrieves the torque reference (in Newton-meters) set for the joint by the system process.
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param joint_torque_ref_ Reference to store the torque reference (in Nm).
         * @return 1 : success, -1 : fail to access data, -2 : joint ID is out of range.
         * @ingroup JointControlAPI
         */
        int getTorqueRef(const int &_jointId, float &joint_torque_ref_);

        /**
         * @brief Overload using JointID.
         * @see getTorqueRef(const int&, float&)
         * @ingroup JointControlAPI
         */
        int getTorqueRef(const JointID _jointId, float &joint_torque_){
            return getTorqueRef(static_cast<int>(_jointId), joint_torque_);
        }

        /**
         * @brief Returns the current position gain (Kp) reference of the specified joint.
         *
         * Retrieves the position gain reference (Kp, in Nm/rad) set for the joint by the system process.
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param joint_kp_ref_ Reference to store the Kp reference (in Nm/rad).
         * @return 1 : success, -1 : fail to access data, -2 : joint ID is out of range.
         * @ingroup JointControlAPI
         */
        int getGainKpRef(const int &_jointId, float &joint_kp_ref_);

        /**
         * @brief Overload using JointID.
         * @see getGainKpRef(const int&, float&)
         * @ingroup JointControlAPI
         */
        int getGainKpRef(const JointID _jointId, float &joint_kp_ref_){
            return getGainKpRef(static_cast<int>(_jointId), joint_kp_ref_);
        }

        /**
         * @brief Returns the current damping gain (Kd) reference of the specified joint.
         *
         * Retrieves the damping gain reference (Kd, in Nm*s/rad) set for the joint by the system process.
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param joint_kd_ref_ Reference to store the Kd reference (in Nm*s/rad).
         * @return 1 : success, -1 : fail to access data, -2 : joint ID is out of range.
         * @ingroup JointControlAPI
         */
        int getGainKdRef(const int &_jointId, float &joint_kd_ref_);

        /**
         * @brief Overload using JointID.
         * @see getGainKdRef(const int&, float&)
         * @ingroup JointControlAPI
         */
        int getGainKdRef(const JointID _jointId, float &joint_kd_ref_){
            return getGainKdRef(static_cast<int>(_jointId), joint_kd_ref_);
        }

        /**
         * @brief Return the current motion owner (process ID) of the specified joint.
         *
         * In a multi-process control architecture, each joint can only be controlled by a single process at a time —
         * the motion owner. This function returns the process ID currently owning the specified joint.
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param joint_processID_ Reference variable to receive the motion owner's process ID.
         *
         * @return Returns 1 on success;
         *         Returns -2 if _jointId is out of range.
         *
         * @ingroup JointControlAPI
         */
        int getMotionOwner(const int &_jointId, int &joint_processID_);

        /**
         * @brief Overload using JointID.
         * @see getMotionOwner(const int&, int&)
         * @ingroup JointControlAPI
         */
        int getMotionOwner(const JointID _jointId, int &joint_processID_){
            return getMotionOwner(static_cast<int>(_jointId), joint_processID_);
        }

        /**
         * @brief Returns the connection status of the specified joint.
         *
         * Retrieves whether the joint is currently connected.
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param connection_status_ Reference to store the connection status (true if connected).
         * @return 1 on success.
         * @ingroup JointControlAPI
         */
        int getConnectionStatus(const int &_jointId, bool &connection_status_);

        /**
         * @brief Overload using JointID.
         * @see getConnectionStatus(const int&, bool&)
         * @ingroup JointControlAPI
         */
        int getConnectionStatus(const JointID _jointId, bool &connection_status_){
            return getConnectionStatus(static_cast<int>(_jointId), connection_status_);
        }

        /**
         * @brief Returns the homing status of the specified joint.
         *
         * Retrieves whether the joint has found home position.
         * homing process is included in Autostart sequence.
         * before finding home position robot need to be put on the flat ground. (see Operation Manual)
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param homming_status_ Reference to store the homing status (true if homed).
         * @return 1 on success.
         * @ingroup JointControlAPI
         */
        int getHomingStatus(const int &_jointId, bool &homming_status_);

        /**
         * @brief Overload using JointID.
         * @see getHomingStatus(const int&, bool&)
         * @ingroup JointControlAPI
         */
        int getHomingStatus(const JointID _jointId, bool &homming_status_){
            return getHomingStatus(static_cast<int>(_jointId), homming_status_);
        }

        /**
         * @brief Returns the run status of the specified joint.
         *
         * Retrieves whether the joint is currently running (motor enabled).
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param run_status_ Reference to store the run status (true if running).
         * @return 1 on success.
         * @ingroup JointControlAPI
         */
        int getRunStatus(const int &_jointId, bool &run_status_);

        /**
         * @brief Overload using JointID.
         * @see getRunStatus(const int&, bool&)
         * @ingroup JointControlAPI
         */
        int getRunStatus(const JointID _jointId, bool &run_status_){
            return getRunStatus(static_cast<int>(_jointId), run_status_);
        }

        /**
         * @brief Returns the motor temperature of the specified joint.
         *
         * Retrieves the current temperature of the joint's motor (in degrees Celsius).
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param motor_temp_ Reference to store the motor temperature (°C).
         * @return 1 on success.
         * @ingroup JointControlAPI
         */
        int getMotorTemperature(const int &_jointId, int &motor_temp_);

        /**
         * @brief Overload using JointID.
         * @see getMotorTemperature(const int&, float&)
         * @ingroup JointControlAPI
         */
        int getMotorTemperature(const JointID _jointId, int &motor_temp_){
            return getMotorTemperature(static_cast<int>(_jointId), motor_temp_);
        }

        /**
         * @brief Returns the board temperature of the specified joint.
         *
         * Retrieves the current temperature of the joint's control board (in degrees Celsius).
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param borad_temp_ Reference to store the board temperature (°C).
         * @return 1 on success.
         * @ingroup JointControlAPI
         */
        int getBoardTemperature(const int &_jointId, int &borad_temp_);

        /**
         * @brief Overload using JointID.
         * @see getBoardTemperature(const int&, float&)
         * @ingroup JointControlAPI
         */
        int getBoardTemperature(const JointID _jointId, int &borad_temp_){
            return getMotorTemperature(static_cast<int>(_jointId), borad_temp_);
        }



    private:
        RBQ_API* m_parent = nullptr;  // RBQ_API class pointer

        MOTION_REF m_motionRef;
    };
    Joint joint{this};

    struct Imu {
        /**
         * @defgroup IMUSensorAPI IMU Sensor API
         * @brief APIs for accessing IMU sensor data such as orientation, angular velocity, and linear acceleration.
         *
         * These functions provide access to the robot's onboard IMU sensor data in different representations,
         * including quaternion, roll-pitch-yaw (RPY), angular velocity (gyro), and linear acceleration.
         *
         * ### Robot Coordinate System
         * - +X: Forward (facing direction of the robot)
         * - +Y: Left
         * - +Z: Upward
         *
         * ### IMU Sensor Placement
         * The IMU sensor is located with an offset from the robot's body frame center:
         * - Offset (in meters): (0.00665, 0.0, -0.0404)
         *
         * All IMU-related data is expressed in this coordinate frame and reflects measurements from this fixed sensor position.
         */

        Imu(RBQ_API* parent) : m_parent(parent) {}

        /**
         * @brief Returns the IMU orientation as a quaternion (w, x, y, z).
         *
         * @param out_imu_ Reference to an Eigen::Quaternion<float> that will be filled with orientation data.
         * @return Returns 1 on success.
         *
         * @ingroup IMUSensorAPI
         */
        int getQuaternion(Eigen::Quaternion<float> &out_imu_);

        /**
         * @brief Returns the IMU orientation as roll, pitch, and yaw angles (in radians).
         *
         * The angles are computed from the quaternion using the Z-Y-X Euler angle convention (yaw → pitch → roll).
         * The output ranges are:
         * - Roll  ∈ [-π, π]
         * - Pitch ∈ [-π/2, π/2]
         * - Yaw   ∈ [-π, π]
         *
         * @param out_rpy_ Reference to a 3D vector that will be filled with (roll, pitch, yaw) in radians.
         * @return Returns 1 on success.
         *
         * @ingroup IMUSensorAPI
         */
        int getRPY(Eigen::Matrix<float, 3, 1> &out_rpy_);

        /**
         * @brief Returns the IMU angular velocity (gyroscope) in rad/s.
         *
         * The output is a 3D vector where each component represents the angular velocity (rate of rotation)
         * around the corresponding axis of the robot:
         * - X: roll rate
         * - Y: pitch rate
         * - Z: yaw rate
         *
         * All values are in radians per second (rad/s).
         * The measurement range is ±2000 deg/s(±34.9 rad/s).
         *
         * @param out_gyro_ Reference to an Eigen::Matrix<float, 3, 1> to store angular velocity.
         * @return Returns 1 on success.
         *
         * @ingroup IMUSensorAPI
         */
        int getGyro(Eigen::Matrix<float, 3, 1> &out_gyro_);

        /**
         * @brief Returns the IMU linear acceleration in m/s².
         *
         * The output is a 3D vector where each component represents the linear acceleration along
         * the corresponding axis of the robot:
         * - X: forward/backward acceleration
         * - Y: lateral (left/right) acceleration
         * - Z: vertical (up/down) acceleration
         *
         * All values are expressed in meters per second squared (m/s²).
         * The typical measurement range is ±16g (±156.96 m/s²).
         *
         * @param out_acc_ Reference to an Eigen::Matrix<float, 3, 1> to store acceleration data.
         * @return Returns 1 on success.
         *
         * @ingroup IMUSensorAPI
         */
        int getAcc(Eigen::Matrix<float, 3, 1> &out_acc_);

        int InitializeImu();

        /**
         * @brief Returns the connection status of the IMU sensor.
         *
         * Retrieves whether the IMU sensor is currently connected.
         *
         * @param connection_status_ Reference to store the connection status (true if connected).
         * @return 1 on success, -1 if shared memory is not accessible.
         * @ingroup IMUSensorAPI
         */
        int getConnectionStatus(bool &connection_status_);

    private:
        RBQ_API* m_parent = nullptr;  // RBQ_API class pointer
    };
    Imu imu{this};

    struct Gamepad {
        /**
         * @defgroup GamepadAPI Gamepad API
         * @brief Access joystick, trigger, and button inputs from the gamepad.
         *
         * This group provides access to the current state of the connected gamepad,
         * including analog joystick positions, trigger values, and digital button states.
         *
         * Joystick jog values and trigger values are normalized to [-1.0, 1.0] or [0.0, 1.0]
         * depending on the axis. Buttons return boolean states.
         *
         * This gamepad mapping is valid for Logitech F710 in 'X' mode
         */

        /**
         * @brief Enumeration of all button types on the gamepad.
         * @ingroup GamepadAPI
         */
        enum class Button {
            A,          ///< Face buttons
            B,          ///< Face buttons
            X,          ///< Face buttons
            Y,          ///< Face buttons
            LB,         ///< Left shoulder buttons
            RB,         ///< Right shoulder buttons
            BACK,       ///< System buttons
            START,      ///< System buttons
            LOGITECH,   ///< Central logo button
            LJOG,       ///< Jog stick clicks
            RJOG,       ///< Jog stick clicks
            AR_UP,      ///< D-pad directions
            AR_DOWN,    ///< D-pad directions
            AR_LEFT,    ///< D-pad directions
            AR_RIGHT,   ///< D-pad directions
        };

        Gamepad(RBQ_API* parent) : m_parent(parent) {}

        /**
         * @brief Gets the left joystick's X-axis input value.
         * @param out_left_jog_x_ Reference to store value in range [-1.0, 1.0].
         * @return 1 on success, 0 if shared memory is not accessible.
         * @ingroup GamepadAPI
         */
        int getLeftJogX(float &out_left_jog_x_);

        /**
         * @brief Gets the left joystick's Y-axis input value.
         * @param out_left_jog_y_ Reference to store value in range [-1.0, 1.0].
         * @return 1 on success, 0 if shared memory is not accessible.
         * @ingroup GamepadAPI
         */
        int getLeftJogY(float &out_left_jog_y_);

        /**
         * @brief Gets the value of the left trigger.
         * @param out_left_trigger_ Reference to store value in range [0.0, 1.0].
         * @return 1 on success, 0 if shared memory is not accessible.
         * @ingroup GamepadAPI
         */
        int getLeftTrigger(float &out_left_trigger_);

        /**
         * @brief Gets the right joystick's X-axis input value.
         * @param out_right_jog_x_ Reference to store value in range [-1.0, 1.0].
         * @return 1 on success, 0 if shared memory is not accessible.
         * @ingroup GamepadAPI
         */
        int getRightJogX(float &out_right_jog_x_);

        /**
         * @brief Gets the right joystick's Y-axis input value.
         * @param out_right_jog_y_ Reference to store value in range [-1.0, 1.0].
         * @return 1 on success, 0 if shared memory is not accessible.
         * @ingroup GamepadAPI
         */
        int getRightJogY(float &out_right_jog_y_);

        /**
         * @brief Gets the value of the right trigger.
         * @param out_right_trigger_ Reference to store value in range [0.0, 1.0].
         * @return 1 on success, 0 if shared memory is not accessible.
         * @ingroup GamepadAPI
         */
        int getRightTrigger(float &out_right_trigger_);

        /**
         * @brief Gets the state of a digital gamepad button.
         * @param _ButtonID The button ID to query.
         * @param out_state_ Reference to store button state (true if pressed).
         * @return 1 on success, 0 if shared memory is not accessible, -1 for invalid Button ID.
         * @ingroup GamepadAPI
         */
        int getButtonState(const Button _ButtonID, bool &out_state_);

        int setGamePad(const float &_left_jog_x, const float &_left_jog_y,
                       const float &_right_jog_x, const float &_right_jog_y,
                       const float &_left_trigger, const float &_right_trigger, const bool _btn[16]);

    private:
        RBQ_API* m_parent = nullptr;  // RBQ_API class pointer

#if defined(PRIVATE)
        JOY_INFO m_joy;
#endif
    };
    Gamepad gamepad{this};

    struct StateEstimation{
        /**
         * @defgroup StateEstimationAPI State Estimation API
         * @brief API for estimating and retrieving the robot's dynamic state information.
         *
         * The StateEstimation class provides real-time estimation of the robot's dynamic states and exposes the results through various getter functions.
         *
         * Main features:
         * - Start and stop state estimation
         * - Set contact detection threshold
         * - Query estimator run state
         * - Retrieve body and foot position/velocity/force/Jacobian in various coordinate frames
         *
         * Returned information includes:
         * - Position and velocity of the robot's body center
         * - Position and velocity of each foot
         * - Jacobian matrix for each foot
         * - Force applied by each foot
         * - External force detected at each foot
         *
         * All information is provided with respect to a specified coordinate frame:
         * - **World**: Fixed to the environment. The origin is the projection of the body center onto the ground at reset. The orientation is: -Z aligned with gravity, +X aligned with the robot's facing direction projected onto the ground at reset.
         * - **Body**: Local body frame. The origin is the body center. Orientation is fixed to the robot body (+X forward, +Z upward, +Y left).
         * - **Body_rp**: Origin at body center. Orientation: -Z aligned with gravity, +X aligned with the robot's facing direction projected onto the ground.
         * - **Body_rpy**: Origin at body center. Orientation matches the global frame.
         *
         * Example:
         * @code
         * // Start state estimation
         * api->stateEstimation.startEstimation();
         *
         * // Get body position in world frame
         * Eigen::Vector3f body_pos;
         * api->stateEstimation.getBodyPos(RBQ_API::StateEstimation::Frame::World, body_pos);
         * @endcode
         */

        /**
         * @brief Definition of Frame of Reference.
         * @ingroup StateEstimationAPI
         */
        enum class Frame : int {
            /**
             * Fixed to the environment. The origin is the projection of the body center onto the ground at reset.
             * Orientation: -Z aligned with gravity, +X aligned with the robot's facing direction projected onto the ground at reset.
             */
            World = 0,

            /**
             * Local body frame. Origin is the body center.
             * Orientation is fixed to the robot body (+X forward, +Z upward, +Y left).
             */
            Body = 1,

            /**
             * Origin at body center.
             * Orientation: -Z aligned with gravity, +X aligned with the robot's facing direction projected onto the ground.
             */
            Body_rp = 2,

            /**
             * Origin at body center.
             * Orientation matches the global frame.
             */
            Body_rpy = 3
        };

        enum class LegID : int {
            HR = 0,  ///< Hind-right leg
            HL = 1,  ///< Hind-left leg
            FR = 2,  ///< Front-right leg
            FL = 3,  ///< Front-left leg
        };

        StateEstimation(RBQ_API* parent) : m_parent(parent) {}

        /**
         * @brief Starts the state estimation process.
         *
         * Resets the coordinate origin to the current body center, with the robot's facing direction set as +X.
         * Once started, the estimator updates the robot's state at 500Hz.
         *
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int startEstimation();

        /**
         * @brief Stops the state estimation process.
         *
         * Halts the estimator and stops updating the robot's state.
         *
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int stopEstimation();

        // z direction external force is used for contact detection
        // this function update update contact detection threshold
        int updateContactThreshold(const float &_contact_threshold);

        /**
         * @brief Returns the current run state of the estimator.
         *
         * Sets @p run_or_not_ to indicate the estimator status:
         * - 0: Estimator is stopped
         * - Non-zero: Estimator is running
         *
         * @param run_or_not_ Reference to receive the estimator run state.
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int getEstimatorRunState(bool &run_or_not_);

        /**
         * @brief Returns the position of the robot's body center.
         *
         * Retrieves the body center position in the specified coordinate frame and stores it in @p body_pos_.
         *
         * @param _frame The coordinate frame for the output position.
         * @param body_pos_ Reference to store the body center position (in meters).
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int getBodyPos(const Frame _frame, Eigen::Vector3f &body_pos_);

        /**
         * @brief Returns the velocity of the robot's body center.
         *
         * Retrieves the body center velocity in the specified coordinate frame and stores it in @p body_vel_.
         *
         * @param _frame The coordinate frame for the output velocity.
         * @param body_vel_ Reference to store the body center velocity (in meters per second).
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int getBodyVel(const Frame _frame, Eigen::Vector3f &body_vel_);

        /**
         * @brief Returns the orientation of the robot's body center as a quaternion.
         *
         * Retrieves the body orientation in the specified coordinate frame and stores it in @p body_quat_.
         *
         * @param _frame The coordinate frame for the output orientation.
         * @param body_quat_ Reference to store the body orientation as a quaternion (w, x, y, z).
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int getBodyQuat(const Frame _frame, Eigen::Quaternion<float> &body_quat_);

        /**
         * @brief Returns the orientation of the robot's body center as roll, pitch, and yaw angles.
         *
         * Retrieves the body orientation in the specified coordinate frame and stores it in @p body_rpy_.
         * The angles are defined by the Euler ZYX convention: yaw → pitch → roll.
         *
         * @param _frame The coordinate frame for the output orientation.
         * @param body_rpy_ Reference to store (roll, pitch, yaw) angles in radians.
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int getBodyRPY(const Frame _frame, Eigen::Vector3f &body_rpy_);

        /**
         * @brief Returns the rotation matrix of the robot's body center.
         *
         * Retrieves the body orientation in the specified coordinate frame and stores it in @p body_rot_.
         * The rotation matrix is a 3x3 matrix representing the orientation of the body center.
         *
         * @param _frame The coordinate frame for the output orientation.
         * @param body_rot_ Reference to store the body rotation matrix (3x3).
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int getBodyRot(const Frame _frame, Eigen::Matrix3f &body_rot_);

        /**
         * @brief Returns the position of a specific foot in the specified coordinate frame.
         *
         * Retrieves the foot position in the specified coordinate frame and stores it in @p foot_pos_.
         *
         * @param _frame The coordinate frame for the output position.
         * @param _legId The leg ID (valid range: 0 to 3).
         * @param foot_pos_ Reference to store the foot position (in meters).
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int getFootPos(const Frame _frame, const int &_legId, Eigen::Vector3f &foot_pos_);

        /**
         * @brief Overload using LegID.
         * @see getFootPos(const Frame, const int&, Eigen::Vector3f&)
         * @ingroup StateEstimationAPI
         */
        int getFootPos(const Frame _frame, const LegID _legId, Eigen::Vector3f &foot_pos_){
            return getFootPos(_frame, static_cast<int>(_legId), foot_pos_);
        }

        /**
         * @brief Returns the velocity of a specific foot in the specified coordinate frame.
         *
         * Retrieves the foot velocity in the specified coordinate frame and stores it in @p foot_vel_.
         *
         * @param _frame The coordinate frame for the output velocity.
         * @param _legId The leg ID (valid range: 0 to 3).
         * @param foot_vel_ Reference to store the foot velocity (in meters per second).
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int getFootVel(const Frame _frame, const int &_legId, Eigen::Vector3f &foot_vel_);

        /**
         * @brief Overload using LegID.
         * @see getFootVel(const Frame, const int&, Eigen::Vector3f&)
         * @ingroup StateEstimationAPI
         */
        int getFootVel(const Frame _frame, const LegID &_legId, Eigen::Vector3f &foot_vel_){
            return getFootVel(_frame, static_cast<int>(_legId), foot_vel_);
        }

        /**
         * @brief Returns the Jacobian matrix of a specific foot in the specified coordinate frame.
         *
         * Retrieves the foot Jacobian in the specified coordinate frame and stores it in @p foot_jacobian_.
         * The Jacobian is a 3x3 matrix representing the relationship between joint velocities and foot velocities.
         *
         * @param _frame The coordinate frame for the output Jacobian.
         * @param _legId The leg ID (valid range: 0 to 3).
         * @param foot_jacobian_ Reference to store the foot Jacobian (3x3 matrix).
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int getFootJacobian(const Frame _frame, const int &_legId, Eigen::Matrix3f &foot_jacobian_);

        /**
         * @brief Overload using LegID.
         * @see getFootJacobian(const Frame, const int&, Eigen::Matrix3f&)
         * @ingroup StateEstimationAPI
         */
        int getFootJacobian(const Frame _frame, const LegID &_legId, Eigen::Matrix3f &foot_jacobian_){
            return getFootJacobian(_frame, static_cast<int>(_legId), foot_jacobian_);
        }

        /**
         * @brief Returns the force applied by a specific foot in the specified coordinate frame.
         *
         * Retrieves the foot force in the specified coordinate frame and stores it in @p foot_force_.
         * The force is expressed in Newtons (N).
         *
         * @param _frame The coordinate frame for the output force.
         * @param _legId The leg ID (valid range: 0 to 3).
         * @param foot_force_ Reference to store the foot force (in Newtons).
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int getFootForce(const Frame _frame, const int &_legId, Eigen::Vector3f &foot_force_);

        /**
         * @brief Overload using LegID.
         * @see getFootForce(const Frame, const int&, Eigen::Vector3f&)
         * @ingroup StateEstimationAPI
         */
        int getFootForce(const Frame _frame, const LegID &_legId, Eigen::Vector3f &foot_force_){
            return getFootForce(_frame, static_cast<int>(_legId), foot_force_);
        }

        /**
         * @brief Returns the external force detected at a specific foot in the specified coordinate frame.
         *
         * Retrieves the external force applied to the foot in the specified coordinate frame and stores it in @p foot_contact_force_.
         * This force is used for contact detection and is expressed in Newtons (N).
         *
         * @param _frame The coordinate frame for the output external force.
         * @param _legId The leg ID (valid range: 0 to 3).
         * @param foot_contact_force_ Reference to store the external force (in Newtons).
         * @return 1 on success.
         * @ingroup StateEstimationAPI
         */
        int getFootExtForce(const Frame _frame, const int &_legId, Eigen::Vector3f &foot_contact_force_);

        /**
         * @brief Overload using LegID.
         * @see getFootExtForce(const Frame, const int&, Eigen::Vector3f&)
         * @ingroup StateEstimationAPI
         */
        int getFootExtForce(const Frame _frame, const LegID &_legId, Eigen::Vector3f &foot_contact_force_){
            return getFootExtForce(_frame, static_cast<int>(_legId), foot_contact_force_);
        }

    private:
        RBQ_API* m_parent = nullptr;  // RBQ_API class pointer
    };
    StateEstimation stateEstimation{this};

    struct PtzCamera {

        /**
         * @defgroup PtzCamera PTZ Camera API
         * @brief APIs for controlling and retrieving the state of the PTZ camera.
         *
         * This group provides functions to get and set the pan, tilt, and zoom of the robot's PTZ camera.
         * Pan and tilt angles are in radians, and zoom is a unitless scalar.
         *
         * ### Coordinate System
         * - Pan: Positive values rotate the camera to the left.
         * - Tilt: Positive values tilt the camera downwards.
         *
         * Example:
         * @code
         * // Get current PTZ state
         * float pan, tilt, zoom;
         * api->ptzCamera.getPanTiltZoom(pan, tilt, zoom);
         *
         * // Set new PTZ state
         * api->ptzCamera.setPanTiltZoom(new_pan, new_tilt, new_zoom);
         * @endcode
         *
         * @ingroup PtzCamera
         */
        PtzCamera(RBQ_API* parent) : m_parent(parent) {}

        /**
         * @brief Returns the pan, tilt angles (in radians) and zoom.
         *
         * Retrieves the current pan and tilt angles of the PTZ camera in radians, along with the zoom level.
         * The pan angle is positive when rotating to the left, and the tilt angle is positive when tilting downwards.
         * The zoom value is a unitless scalar representing the zoom level.
         *
         * @param out_pan_ Reference to store the pan angle (in radians).
         * @param out_tilt_ Reference to store the tilt angle (in radians).
         * @param out_zoom_ Reference to store the zoom level (unitless).
         * @return Returns 1 on success.
         *
         * @ingroup PtzCamera
         */
        int getPanTiltZoom(float &out_pan_, float &out_tilt_, float &out_zoom_);

        /**
         * @brief Sets the pan, tilt angles (in radians) and zoom.
         *
         * Sets the desired pan and tilt angles of the PTZ camera in radians, along with the zoom level.
         * The pan angle is positive when rotating to the left, and the tilt angle is positive when tilting downwards.
         * The zoom value is a unitless scalar representing the zoom level.
         *
         * @param _pan The desired pan angle (in radians).
         * @param _tilt The desired tilt angle (in radians).
         * @param _zoom The desired zoom level (unitless).
         * @return Returns 1 on success;
         *
         * @ingroup PtzCamera
         */
        int setPanTiltZoom(const float &_pan, const float &_tilt, const float &_zoom);

    private:
        RBQ_API* m_parent = nullptr;

        float m_pan   = 0.0f;   // radians, positive to left
        float m_tilt  = 0.0f;   // radians, positive to down
        float m_zoom  = 1.0f;   // unitless
    };
    PtzCamera ptzCamera{this};

    struct Simulator {
        /**
         * @defgroup SimulatorAPI Simulator API
         * @brief APIs for interacting with the robot simulator.
         *
         * This group provides functions to check if the robot is running in a simulator environment,
         * and to retrieve the simulation time.
         *
         * Example:
         * @code
         * // Check if running in simulator
         * bool isSim;
         * api->simulator.isSimulator(isSim);
         *
         * // Get current robot position
         * Eigen::Vector3f bodyPos;
         * api->simulator.getBodyPos(bodyPos);
         * @endcode
         *
         * @ingroup SimulatorAPI
         */

        enum class LegID : int {
            HR = 0,  ///< Hind-right leg
            HL = 1,  ///< Hind-left leg
            FR = 2,  ///< Front-right leg
            FL = 3,  ///< Front-left leg
        };

        Simulator(RBQ_API* parent) : m_parent(parent) {}

        /**
         * @brief Checks if the robot is running in a simulator environment.
         * @param is_simulator_ Reference to store the result (true if in simulator).
         * @return 1 on success.
         * @ingroup SimulatorAPI
         */
        int isSimulator(bool &is_simulator_);

        /**
         * @brief Returns the position of the robot's body center.
         *
         * Retrieves the body center position in the world coordinate frame and stores it in @p body_pos_.
         *
         * @param body_pos_ Reference to store the body center position (in meters).
         * @return 1 on success.
         * @ingroup SimulatorAPI
         */
        int getBodyPos(Eigen::Vector3f &body_pos_);

        /**
         * @brief Returns the orientation of the robot's body center as a quaternion.
         *
         * Retrieves the body orientation in the world coordinate frame and stores it in @p body_quat_.
         *
         * @param body_quat_ Reference to store the body orientation as a quaternion (w, x, y, z).
         * @return 1 on success.
         * @ingroup SimulatorAPI
         */
        int getBodyQuat(Eigen::Quaternion<float> &body_quat_);

        int getImuQuat(Eigen::Quaternion<float> &imu_quat_);

        int getImuAcc(Eigen::Vector3f &imu_acc_);

        int getImuGyro(Eigen::Vector3f &imu_gyro_);

        /**
         * @brief Returns the velocity of the robot's body center.
         *
         * Retrieves the body center velocity in the world coordinate frame and stores it in @p body_vel_linear_ and @p body_vel_angular_.
         *
         * @param body_vel_linear_ Reference to store the linear velocity (in meters per second).
         * @param body_vel_angular_ Reference to store the angular velocity (in radians per second).
         * @return 1 on success.
         * @ingroup SimulatorAPI
         */
        int getBodyVel(Eigen::Vector3f &body_vel_linear_, Eigen::Vector3f &body_vel_angular_);

        /**
         * @brief Returns the current simulation time in seconds.
         * @param simulation_time_ Reference to store the simulation time (in seconds).
         * @return 1 on success.
         * @ingroup SimulatorAPI
         */
        int getTime(double &simulation_time_);

        /**
         * @brief Returns the position of the specified joint.
         *
         * Retrieves the current position of the specified joint in radians.
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param joint_pos_ Reference to store the joint position (in radians).
         * @return 1 on success.
         * @ingroup SimulatorAPI
         */
        int getJointPos(const int &_jointId, float &joint_pos_);

        /**
         * @brief Returns the velocity of the specified joint.
         *
         * Retrieves the current velocity of the specified joint in radians per second.
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param joint_vel_ Reference to store the joint velocity (in radians per second).
         * @return 1 on success.
         * @ingroup SimulatorAPI
         */
        int getJointVel(const int &_jointId, float &joint_vel_);

        /**
         * @brief Returns the torque of the specified joint.
         *
         * Retrieves the current torque of the specified joint in Newton-meters.
         *
         * @param _jointId The joint ID (valid range: 0 to 11, wheel: 12 to 15).
         * @param joint_torque_ Reference to store the joint torque (in Newton-meters).
         * @return 1 on success.
         * @ingroup SimulatorAPI
         */
        int getJointTorque(const int &_jointId, float &joint_torque_);

        /**
         * @brief Returns the contact force at the specified leg's foot.
         *
         * Retrieves the contact force vector applied at the foot of the specified leg in Newtons.
         *
         * @param _legId The leg ID (valid range: 0 to 3).
         * @param contact_force_ Reference to store the contact force vector (in Newtons).
         * @return 1 on success.
         * @ingroup SimulatorAPI
         */
        int getFootContactForce(const int &_legId, Eigen::Vector3f &contact_force_);

        /**
         * @brief Overload using LegID.
         * @see getFootContactForce(const int&, Eigen::Vector3f&)
         * @ingroup SimulatorAPI
         */
        int getFootContactForce(const LegID &_legId, Eigen::Vector3f &contact_force_) {
            return getFootContactForce(static_cast<int>(_legId), contact_force_);
        }

        /**
         * @brief Returns whether the specified leg is in contact with the ground.
         *
         * Determines if the foot of the specified leg is currently in contact with the ground.
         *
         * @param _legId The leg ID (valid range: 0 to 3).
         * @param contact_ Reference to store the contact status (true if in contact).
         * @return 1 on success.
         * @ingroup SimulatorAPI
         */
        int getFootContact(const int &_legId, bool &contact_);

        /**
         * @brief Overload using LegID.
         * @see getFootContact(const int&, bool&)
         * @ingroup SimulatorAPI
         */
        int getFootContact(const LegID &_legId, bool &contact_) {
            return getFootContact(static_cast<int>(_legId), contact_);
        }

    private:
        RBQ_API* m_parent = nullptr;
    };
    Simulator simulator{this};

#if defined(PRIVATE)
    struct Command {
        Command(RBQ_API* parent) : m_parent(parent) {

        }

        int getUserCommand(int &out_command_);
        int getUserParaInt(const int &_index, int &out_parameter_);
        int getUserParaChar(const int &_index, char &out_parameter_);
        int getUserParaFloat(const int &_index, float &out_parameter_);
        int getUserParaDouble(const int &_index, double &out_parameter_);

        int setUserCommand(const int &_set_command);
        int setUserParaInt(const int &_index, const int &_set_parameter);
        int setUserParaChar(const int &_index, const char &_set_parameter);
        int setUserParaFloat(const int &_index, const float &_set_parameter);
        int setUserParaDouble(const int &_index, const double &_set_parameter);

        int getTotalCommand(COMMAND_STRUCT &out_command_structure_);
        COMMAND_STRUCT getTotalCommand(const int &_target_processId);

        int sendCommand_internal(const int &_target_processID);
        int sendCommand_tcp(const int &_target_processID);


    private:
        RBQ_API* m_parent = nullptr;  // RBQ_API class pointer

        COMMAND_STRUCT m_command;
        int* m_socket = nullptr;
    };
    Command command{this};

    void setCommand(const COMMAND_STRUCT &cmd);

    int setCommand(const HIGH_LEVEL_CMD &cmd);

    void setCommand(const HighLevelCmd_t &cmd);

    int setCommand(const RBQ_SDK::GeneralRequest_t &cmd);
private:
    pRBCORE_SHM_COMMAND     m_sharedCMD            = nullptr;
    pRBCORE_SHM_REFERENCE   m_sharedREF            = nullptr;
    pRBCORE_SHM_SENSOR      m_sharedSEN            = nullptr;
    pUSER_SHM               m_sharedUSER           = nullptr;
    int                     m_shmOpened            = 0;
    int _openSHM();
    pRBCORE_SHM_SENSOR _getSensorData();
    pRBCORE_SHM_REFERENCE _getRefData();
    pRBCORE_SHM_COMMAND _getCmdData();
    pUSER_SHM           _getUserData();
    SIM_VARIABLE        _getSimInfo() const;
    MOTION_REF          _getMotionRef() const;

    bool m_shm      = true;
    bool m_shm_ref  = true;

    int m_processId = -1;

private:
    template <typename T>
    static const T& clamp(const T& value, const T& low, const T& high)
    {
        return (value < low) ? low : (value > high) ? high : value;
    }

    static constexpr float FLOAT_EPSILON = 1e-4f;
    static bool floatEquals(float a, float b)
    {
        return std::abs(a - b) <= FLOAT_EPSILON;
    }


#endif // PRIVATE

private:
    RBQ_API();
    ~RBQ_API();
    static bool         m_initialized;
    static std::mutex   m_mutex;

    friend class StateEstimator;
};

#endif // RBQ_API_H
