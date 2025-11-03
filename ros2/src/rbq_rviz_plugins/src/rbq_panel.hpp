#ifndef RBQ_RVIZ_PLUGINS__NAV2_PANEL_HPP_
#define RBQ_RVIZ_PLUGINS__NAV2_PANEL_HPP_

#include <QtWidgets>
#include <QBasicTimer>
#include <QTimer>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rviz_common/panel.hpp"
#include "rbq_msgs/msg/robot_status.hpp"
#include "rbq_msgs/msg/battery_state.hpp"
#include "rbq_msgs/msg/joint_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>

class QPushButton;

namespace rbq_rviz_plugins
{

/// Panel to interface to the nav2 stack
class RbqPanel : public rviz_common::Panel
{
    Q_OBJECT

public:
    explicit RbqPanel(QWidget * parent = 0);
    virtual ~RbqPanel();

    void onInitialize() override;

    /// Load and save configuration data
    void load(const rviz_common::Config & config) override;
    void save(rviz_common::Config config) const override;

private Q_SLOTS:
    void startThread();
    void onStartup();
    void onShutdown();
    void onCancel();
    void onPause();
    void onResume();
    void onAccumulatedWp();
    void onAccumulatedNTP();
    void onAccumulating();
    void onNewGoal(double x, double y, double theta, QString frame);

    // void onAutoStart       ();
    // void onStand           ();
    // void onSit             ();

    void autoStart         ();
    void canCheck          ();
    void findHome          ();
    void sit               ();
    void stand             ();
    void walk              ();
    void walkSlow          ();
    void run               ();
    void dock              ();
    void emergency         ();
    
    // RL 관련 함수들
    void stairs            ();
    void rlTrot            ();
    void rlBound           ();
    void rlPace            ();
    void rlPronk           ();
    void rlTrotVision      ();
    void rlTrotRun         ();
    void rlSilent          ();
    void rlFrontWalk       ();

    void switchGamepadPort (const bool &powerON = false);
    void extJoyToggle      ();
    void powerLeg          (const bool &powerON = false);
    void powerArm          (const bool &powerON = false);
    void powerVisionPC     (const bool &powerON = false);
    void powerUsbHub       (const bool &powerON = false);
    void powerCctv         (const bool &powerON = false);
    void powerThermal      (const bool &powerON = false);
    void powerLidar        (const bool &powerON = false);
    void powerExt52V       (const bool &powerON = false);
    void powerIrLEDs       (const bool &powerON = false);
    void powerComm         (const bool &powerON = false);
    void setBodyHeight     (const int  &height  = false);
    void setFootHeight     (const int  &height  = false);
    void setMaxSpeed       (const int  &speed   = false);
    void comEstimation     (const int  &stage   = -1);

private:
    void loadLogFiles();
    void onCancelButtonPressed();
    void timerEvent(QTimerEvent * event) override;

    int unique_id {0};

    // A timer used to check on the completion status of the action
    QBasicTimer timer_;

    QPushButton* bt_autoStart          = nullptr;
    QPushButton* bt_canCheck           = nullptr;
    QPushButton* bt_findHome           = nullptr;
    QPushButton* bt_sit                = nullptr;
    QPushButton* bt_stand              = nullptr;
    QPushButton* bt_walk               = nullptr;
    QPushButton* bt_walkSlow           = nullptr;
    QPushButton* bt_run                = nullptr;
    
    // RL 관련 버튼들
    QPushButton* bt_stairs             = nullptr;
    QPushButton* bt_rlTrot             = nullptr;
    QPushButton* bt_rlBound            = nullptr;
    QPushButton* bt_rlPace             = nullptr;
    QPushButton* bt_rlPronk            = nullptr;
    QPushButton* bt_rlTrotVision       = nullptr;
    QPushButton* bt_rlTrotRun          = nullptr;
    QPushButton* bt_rlSilent           = nullptr;
    QPushButton* bt_rlFrontWalk        = nullptr;
    QPushButton* bt_dock               = nullptr;
    QPushButton* bt_emergency          = nullptr;
    QPushButton* bt_switchGamepadPort  = nullptr;
    QPushButton* bt_extJoy             = nullptr;
    QPushButton* bt_powerLeg           = nullptr;
    QPushButton* bt_powerArm           = nullptr;
    QPushButton* bt_powerVisionPC      = nullptr;
    QPushButton* bt_powerUsbHub        = nullptr;
    QPushButton* bt_powerCctv          = nullptr;
    QPushButton* bt_powerThermal       = nullptr;
    QPushButton* bt_powerLidar         = nullptr;
    QPushButton* bt_powerExt52V        = nullptr;
    QPushButton* bt_powerIrLEDs        = nullptr;
    QPushButton* bt_powerComm          = nullptr;
    QPushButton* bt_setBodyHeight      = nullptr;
    QPushButton* bt_setFootHeight      = nullptr;
    QPushButton* bt_setMaxSpeed        = nullptr;
    QPushButton* bt_comEstimation      = nullptr;

    QPushButton* start_reset_button_{nullptr};
    QPushButton* pause_resume_button_{nullptr};
    QPushButton* navigation_mode_button_{nullptr};

    QLabel* navigation_status_indicator_{nullptr};
    QLabel* localization_status_indicator_{nullptr};
    QLabel* navigation_goal_status_indicator_{nullptr};
    QLabel* navigation_feedback_indicator_{nullptr};

    QStateMachine state_machine_;

    QState* pre_initial_{nullptr};
    QState* initial_{nullptr};
    QState* idle_{nullptr};
    QState* reset_{nullptr};
    QState* paused_{nullptr};
    QState* resumed_{nullptr};
    // The following states are added to allow for the state of the button to only expose reset
    // while the NavigateToPoses action is not active. While running, the user will be allowed to
    // cancel the action. The ROSActionTransition allows for the state of the action to be detected
    // and the button state to change automatically.
    QState* running_{nullptr};
    QState* canceled_{nullptr};
    // The following states are added to allow to collect several poses to perform a waypoint-mode
    // navigation or navigate through poses mode.
    QState* accumulating_{nullptr};
    QState* accumulated_wp_{nullptr};
    QState* accumulated_nav_through_poses_{nullptr};

    // Publish the visual markers with the waypoints
    void updateWpNavigationMarkers();

    // Create unique id numbers for markers
    int getUniqueId();

    void resetUniqueId();

    // round off double to the specified precision and convert to string
    static inline std::string toString(double val, int precision = 0);

    template<typename T>
    static inline std::string toLabel(T & msg);

    // Status indicators
    QLabel* lbl_battery_voltage;
    QLabel* lbl_motor_check_status;
    QLabel* lbl_initialize_pose_status;
    QLabel* lbl_control_start_status;
    QLabel* lbl_imu_status;
    
    // ROS2 node for subscriptions
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread executor_thread_;
    
    // ROS2 subscriptions
    rclcpp::Subscription<rbq_msgs::msg::RobotStatus>::SharedPtr status_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
    rclcpp::Subscription<rbq_msgs::msg::JointStatus>::SharedPtr joint_status_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    
    // Callback functions
    void statusCallback(const rbq_msgs::msg::RobotStatus::SharedPtr msg);
    void batteryStateCallback(const rbq_msgs::msg::BatteryState::SharedPtr msg);
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void jointStatusCallback(const rbq_msgs::msg::JointStatus::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    // Executor thread function
    void executorThreadFunction();
    
    // Gait button color update function
    void updateGaitButtonColors(int currentGaitId);
    
    // Ext joy button color update function
    void updateExtJoyButtonColor(bool extJoyConnected);
    
    // Joint table update function
    void updateJointTable();
    
    // Odometry table update function
    void updateOdometryTable();
    
    // IMU table update function
    void updateImuTable();
    
    // Tab widget
    QTabWidget* tab_widget_;
    
    // Tab creation functions
    QWidget* createMainTab();
    QWidget* createAdvancedTab();
    
    // Update timer for batch table updates (like GUI)
    QTimer* update_timer_;
    
    // Data storage for batch updates
    struct RobotData {
        rbq_msgs::msg::RobotStatus::SharedPtr robot_status;
        sensor_msgs::msg::JointState::SharedPtr joint_states;
        rbq_msgs::msg::JointStatus::SharedPtr joint_status;
        nav_msgs::msg::Odometry::SharedPtr odometry;
        sensor_msgs::msg::Imu::SharedPtr imu;
        rbq_msgs::msg::BatteryState::SharedPtr battery_state;
        
        std::mutex data_mutex;
    } robot_data_;
    
    
    // HighLevel 상태 변수
    bool highLevel_state_;
    
    // Batch update function
    void batchUpdateTables();
};

}  // namespace rbq_rviz_plugins

#endif  //  RBQ_RVIZ_PLUGINS__NAV2_PANEL_HPP_
