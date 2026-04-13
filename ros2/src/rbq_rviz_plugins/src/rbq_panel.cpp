#include "rbq_panel.hpp"

#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include <memory>
#include <vector>
#include <utility>
#include <chrono>
#include <string>

#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "Publisher.h"

std::shared_ptr<Publisher> publisher;

using namespace std::chrono_literals;

namespace rbq_rviz_plugins
{

RbqPanel::RbqPanel(QWidget * parent)
    : Panel(parent)
{
    // ExtJoy 상태 초기화 (OFF)
    highLevel_state_ = false;
    
    // Create the control button and its tooltip

    start_reset_button_ = new QPushButton;
    pause_resume_button_ = new QPushButton;
    navigation_mode_button_ = new QPushButton;
    navigation_status_indicator_ = new QLabel;
    localization_status_indicator_ = new QLabel;
    navigation_goal_status_indicator_ = new QLabel;
    navigation_feedback_indicator_ = new QLabel;

    publisher = std::make_shared<Publisher>("rbq_rviz_panel");

    bt_autoStart          = new QPushButton;
    bt_canCheck           = new QPushButton;
    bt_findHome           = new QPushButton;
    bt_sit                = new QPushButton;
    bt_stand              = new QPushButton;
    bt_walk               = new QPushButton;
    bt_walkSlow           = new QPushButton;
    bt_run                = new QPushButton;
    bt_dock               = new QPushButton;
    
    // RL 관련 버튼들 생성
    bt_stairs             = new QPushButton;
    bt_rlTrot             = new QPushButton;
    bt_rlBound            = new QPushButton;
    bt_rlPace             = new QPushButton;
    bt_rlPronk            = new QPushButton;
    bt_rlTrotVision       = new QPushButton;
    bt_rlTrotRun          = new QPushButton;
    bt_rlSilent           = new QPushButton;
    bt_rlFrontWalk        = new QPushButton;
    bt_emergency          = new QPushButton;
    bt_extJoy             = new QPushButton;
    bt_powerLeg           = new QPushButton;
    bt_powerArm           = new QPushButton;
    bt_powerVisionPC      = new QPushButton;
    bt_powerUsbHub        = new QPushButton;
    bt_powerCctv          = new QPushButton;
    bt_powerThermal       = new QPushButton;
    bt_powerLidar         = new QPushButton;
    bt_powerExt52V        = new QPushButton;
    bt_powerIrLEDs        = new QPushButton;
    bt_powerComm          = new QPushButton;
    bt_setBodyHeight      = new QPushButton;
    bt_setFootHeight      = new QPushButton;
    bt_setMaxSpeed        = new QPushButton;
    bt_comEstimation      = new QPushButton;

    // Status indicators - NO CIRCLE SYMBOL
    lbl_battery_voltage = new QLabel("0.0 V");
    lbl_motor_check_status = new QLabel("");
    lbl_initialize_pose_status = new QLabel("");
    lbl_control_start_status = new QLabel("");
    lbl_imu_status = new QLabel("");
    
    // Status indicator styling - RECTANGULAR INDICATORS
    lbl_battery_voltage->setStyleSheet("QLabel { background-color: white; border: 1px solid #ccc; padding: 5px; font-weight: bold; }");
    lbl_motor_check_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
    lbl_initialize_pose_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
    lbl_control_start_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
    lbl_imu_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");


    bt_autoStart         ->setText( "Auto\nStart"        );
    bt_canCheck          ->setText( "Can\nCheck"         );
    bt_findHome          ->setText( "Find\nHome"         );
    bt_sit               ->setText( "Sit"                );
    bt_stand             ->setText( "Stand"              );
    bt_walk              ->setText( "Walk"               );
    bt_walkSlow          ->setText( "Wave"         );
    bt_run               ->setText( "Run"                );
    bt_emergency         ->setText( "Emergency"          );
    bt_emergency         ->setStyleSheet("QPushButton { background-color: #ff4444; color: white; font-weight: bold; font-size: 14px; border: 2px solid #cc0000; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #ff6666; } QPushButton:pressed { background-color: #cc0000; }");
    
    // RL 관련 버튼 텍스트 설정
    bt_stairs            ->setText( "Stairs"             );
    bt_rlTrot            ->setText( "RL\nTrot"           );
    bt_rlBound           ->setText( "RL\nBound"          );
    bt_rlPace            ->setText( "RL\nPace"           );
    bt_rlPronk           ->setText( "RL\nPronk"          );
    bt_rlTrotVision      ->setText( "RL\nTrot Vision"   );
    bt_rlTrotRun         ->setText( "RL\nTrot Run"      );
    bt_rlSilent          ->setText( "RL\nSilent"         );
    bt_rlFrontWalk       ->setText( "RL\nFront Walk"    );
    
    // Gait Select 버튼들 초기 스타일 설정 (기본 회색)
    QString defaultGaitStyle = "QPushButton { background-color: #f0f0f0; color: #333333; font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #e0e0e0; } QPushButton:pressed { background-color: #d0d0d0; }";
    bt_sit               ->setStyleSheet(defaultGaitStyle);
    bt_stand             ->setStyleSheet(defaultGaitStyle);
    bt_walk              ->setStyleSheet(defaultGaitStyle);
    bt_walkSlow          ->setStyleSheet(defaultGaitStyle);
    bt_run               ->setStyleSheet(defaultGaitStyle);
    
    bt_stairs            ->setStyleSheet(defaultGaitStyle);
    bt_rlTrot            ->setStyleSheet(defaultGaitStyle);
    bt_rlBound           ->setStyleSheet(defaultGaitStyle);
    bt_rlPace            ->setStyleSheet(defaultGaitStyle);
    bt_rlPronk           ->setStyleSheet(defaultGaitStyle);
    bt_rlTrotVision      ->setStyleSheet(defaultGaitStyle);
    bt_rlTrotRun         ->setStyleSheet(defaultGaitStyle);
    bt_rlSilent          ->setStyleSheet(defaultGaitStyle);
    bt_rlFrontWalk       ->setStyleSheet(defaultGaitStyle);
    bt_extJoy            ->setText( "HighLevel\nControl"           );
    bt_powerLeg          ->setText( "Leg"         );
    bt_powerArm          ->setText( "Arm"         );
    bt_powerVisionPC     ->setText( "VisionPC"    );
    bt_powerUsbHub       ->setText( "UsbHub"      );
    bt_powerCctv         ->setText( "Cctv"        );
    bt_powerThermal      ->setText( "Thermal"     );
    bt_powerLidar        ->setText( "Lidar"       );
    bt_powerExt52V       ->setText( "Ext52V"      );
    bt_powerIrLEDs       ->setText( "IrLEDs"      );
    bt_powerComm         ->setText( "Comm"        );
    bt_setBodyHeight     ->setText( "Set Body Height"    );
    bt_setFootHeight     ->setText( "Set Foot Height"    );
    bt_setMaxSpeed       ->setText( "Set Max Speed"      );
    bt_comEstimation     ->setText( "CoM\nEstimation"     );
    bt_dock              ->setText( "Dock"                );

    bt_extJoy            ->setCheckable(true);
    bt_powerLeg          ->setCheckable(true);
    bt_powerArm          ->setCheckable(true);
    bt_powerVisionPC     ->setCheckable(true);
    bt_powerUsbHub       ->setCheckable(true);
    bt_powerCctv         ->setCheckable(true);
    bt_powerThermal      ->setCheckable(true);
    bt_powerLidar        ->setCheckable(true);
    bt_powerExt52V       ->setCheckable(true);
    bt_powerIrLEDs       ->setCheckable(true);
    bt_powerComm         ->setCheckable(true);

    bt_extJoy            ->setChecked(false);
    bt_powerLeg          ->setChecked(false);
    bt_powerArm          ->setChecked(false);
    bt_powerVisionPC     ->setChecked(false);
    bt_powerUsbHub       ->setChecked(false);
    bt_powerCctv         ->setChecked(false);
    bt_powerThermal      ->setChecked(false);
    bt_powerLidar        ->setChecked(false);
    bt_powerExt52V       ->setChecked(false);
    bt_powerIrLEDs       ->setChecked(false);
    bt_powerComm         ->setChecked(false);

    QObject::connect(bt_autoStart         , &QPushButton::clicked, this, &RbqPanel::autoStart         );
    QObject::connect(bt_canCheck          , &QPushButton::clicked, this, &RbqPanel::canCheck          );
    QObject::connect(bt_findHome          , &QPushButton::clicked, this, &RbqPanel::findHome          );
    QObject::connect(bt_sit               , &QPushButton::clicked, this, &RbqPanel::sit               );
    QObject::connect(bt_stand             , &QPushButton::clicked, this, &RbqPanel::stand             );
    QObject::connect(bt_walk              , &QPushButton::clicked, this, &RbqPanel::walk              );
    QObject::connect(bt_walkSlow          , &QPushButton::clicked, this, &RbqPanel::walkSlow          );
    QObject::connect(bt_run               , &QPushButton::clicked, this, &RbqPanel::run               );
    QObject::connect(bt_dock              , &QPushButton::clicked, this, &RbqPanel::dock              );
    
    // RL 관련 버튼들 연결
    QObject::connect(bt_stairs            , &QPushButton::clicked, this, &RbqPanel::stairs            );
    QObject::connect(bt_rlTrot            , &QPushButton::clicked, this, &RbqPanel::rlTrot            );
    QObject::connect(bt_rlBound           , &QPushButton::clicked, this, &RbqPanel::rlBound           );
    QObject::connect(bt_rlPace            , &QPushButton::clicked, this, &RbqPanel::rlPace            );
    QObject::connect(bt_rlPronk           , &QPushButton::clicked, this, &RbqPanel::rlPronk           );
    QObject::connect(bt_rlTrotVision      , &QPushButton::clicked, this, &RbqPanel::rlTrotVision      );
    QObject::connect(bt_rlTrotRun         , &QPushButton::clicked, this, &RbqPanel::rlTrotRun         );
    QObject::connect(bt_rlSilent          , &QPushButton::clicked, this, &RbqPanel::rlSilent          );
    QObject::connect(bt_rlFrontWalk       , &QPushButton::clicked, this, &RbqPanel::rlFrontWalk       );
    QObject::connect(bt_emergency         , &QPushButton::clicked, this, &RbqPanel::emergency         );
    QObject::connect(bt_extJoy            , &QPushButton::clicked, this, &RbqPanel::extJoyToggle      );
    QObject::connect(bt_powerLeg          , &QPushButton::clicked, this, &RbqPanel::powerLeg          );
    QObject::connect(bt_powerArm          , &QPushButton::clicked, this, &RbqPanel::powerArm          );
    QObject::connect(bt_powerVisionPC     , &QPushButton::clicked, this, &RbqPanel::powerVisionPC     );
    QObject::connect(bt_powerUsbHub       , &QPushButton::clicked, this, &RbqPanel::powerUsbHub       );
    QObject::connect(bt_powerCctv         , &QPushButton::clicked, this, &RbqPanel::powerCctv         );
    QObject::connect(bt_powerThermal      , &QPushButton::clicked, this, &RbqPanel::powerThermal      );
    QObject::connect(bt_powerLidar        , &QPushButton::clicked, this, &RbqPanel::powerLidar        );
    QObject::connect(bt_powerExt52V       , &QPushButton::clicked, this, &RbqPanel::powerExt52V       );
    QObject::connect(bt_powerIrLEDs       , &QPushButton::clicked, this, &RbqPanel::powerIrLEDs       );
    QObject::connect(bt_powerComm         , &QPushButton::clicked, this, &RbqPanel::powerComm         );
    // QObject::connect(bt_setBodyHeight     , &QPushButton::clicked, this, &RbqPanel::setBodyHeight     );
    // QObject::connect(bt_setFootHeight     , &QPushButton::clicked, this, &RbqPanel::setFootHeight     );
    // QObject::connect(bt_setMaxSpeed       , &QPushButton::clicked, this, &RbqPanel::setMaxSpeed       );
    QObject::connect(bt_comEstimation     , &QPushButton::clicked, this, &RbqPanel::comEstimation     );

    // Create ROS2 node for subscriptions - SAFER IMPLEMENTATION
    try {
        ros_node_ = rclcpp::Node::make_shared("rbq_panel_node_" + std::to_string(getpid()));
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(ros_node_);
        
        // ROS2 subscriptions
        status_subscription_ = ros_node_->create_subscription<rbq_msgs::msg::RobotStatus>(
            "rbq/status/robot_status", 10, 
            std::bind(&RbqPanel::statusCallback, this, std::placeholders::_1));
        
           joint_states_subscription_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
               "rbq/joint_states", 10, 
               std::bind(&RbqPanel::jointStatesCallback, this, std::placeholders::_1));
           
           joint_status_subscription_ = ros_node_->create_subscription<rbq_msgs::msg::JointStatus>(
               "rbq/joint/joint_status", 10, 
               std::bind(&RbqPanel::jointStatusCallback, this, std::placeholders::_1));
           
           odometry_subscription_ = ros_node_->create_subscription<nav_msgs::msg::Odometry>(
               "rbq/stateEstimation/odometry", 10, 
               std::bind(&RbqPanel::odometryCallback, this, std::placeholders::_1));
           
           imu_subscription_ = ros_node_->create_subscription<sensor_msgs::msg::Imu>(
               "rbq/imu/IMU_state", 10, 
               std::bind(&RbqPanel::imuCallback, this, std::placeholders::_1));
        
        // Start executor in separate thread
        executor_thread_ = std::thread(&RbqPanel::executorThreadFunction, this);
        
        RCLCPP_INFO(ros_node_->get_logger(), "ROS2 subscriptions initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rbq_panel"), "Failed to initialize ROS2 subscriptions: %s", e.what());
        // Continue without subscriptions
    }

    // Create tab widget
    tab_widget_ = new QTabWidget();
    
    // Create tabs
    QWidget* main_tab = createMainTab();
    QWidget* advanced_tab = createAdvancedTab();
    
    // Add tabs to tab widget
    tab_widget_->addTab(main_tab, "Main Control");
    tab_widget_->addTab(advanced_tab, "State");
    
    // Initialize update timer for batch table updates (like GUI)
    update_timer_ = new QTimer(this);
    update_timer_->setInterval(50); // 50ms like GUI
    connect(update_timer_, &QTimer::timeout, this, &RbqPanel::batchUpdateTables);
    update_timer_->start();
    
    QVBoxLayout * lyt_buttons = new QVBoxLayout;
    lyt_buttons->addWidget(tab_widget_);


    

    {


    // Create the state machine used to present the proper control button states in the UI

    const char * startup_msg = "Configure and activate all nav2 lifecycle nodes";
    const char * shutdown_msg = "Deactivate and cleanup all nav2 lifecycle nodes";
    const char * cancel_msg = "Cancel navigation";
    const char * pause_msg = "Deactivate all nav2 lifecycle nodes";
    const char * resume_msg = "Activate all nav2 lifecycle nodes";
    const char * single_goal_msg = "Change to waypoint / nav through poses style navigation";
    const char * waypoint_goal_msg = "Start following waypoints";
    const char * nft_goal_msg = "Start navigating through poses";
    const char * cancel_waypoint_msg = "Cancel waypoint / viapoint accumulation mode";

    const QString navigation_active("<table><tr><td width=100><b>Navigation:</b></td>"
                                    "<td><font color=green>active</color></td></tr></table>");
    const QString navigation_inactive("<table><tr><td width=100><b>Navigation:</b></td>"
                                      "<td>inactive</td></tr></table>");
    const QString navigation_unknown("<table><tr><td width=100><b>Navigation:</b></td>"
                                     "<td>unknown</td></tr></table>");
    const QString localization_active("<table><tr><td width=100><b>Localization:</b></td>"
                                      "<td><font color=green>active</color></td></tr></table>");
    const QString localization_inactive("<table><tr><td width=100><b>Localization:</b></td>"
                                        "<td>inactive</td></tr></table>");
    const QString localization_unknown("<table><tr><td width=100><b>Localization:</b></td>"
                                       "<td>unknown</td></tr></table>");

    navigation_status_indicator_->setText(navigation_unknown);
    localization_status_indicator_->setText(localization_unknown);
    navigation_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    localization_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    navigation_goal_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    navigation_feedback_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    pre_initial_ = new QState();
    pre_initial_->setObjectName("pre_initial");
    pre_initial_->assignProperty(start_reset_button_, "text", "Startup");
    pre_initial_->assignProperty(start_reset_button_, "enabled", false);

    pre_initial_->assignProperty(pause_resume_button_, "text", "Pause");
    pre_initial_->assignProperty(pause_resume_button_, "enabled", false);

    pre_initial_->assignProperty(
        navigation_mode_button_, "text",
        "Waypoint / Nav Through Poses Mode");
    pre_initial_->assignProperty(navigation_mode_button_, "enabled", false);

    initial_ = new QState();
    initial_->setObjectName("initial");
    initial_->assignProperty(start_reset_button_, "text", "Startup");
    initial_->assignProperty(start_reset_button_, "toolTip", startup_msg);
    initial_->assignProperty(start_reset_button_, "enabled", true);

    initial_->assignProperty(pause_resume_button_, "text", "Pause");
    initial_->assignProperty(pause_resume_button_, "enabled", false);

    initial_->assignProperty(navigation_mode_button_, "text", "Waypoint / Nav Through Poses Mode");
    initial_->assignProperty(navigation_mode_button_, "enabled", false);

    // State entered when navigate_to_pose action is not active
    idle_ = new QState();
    idle_->setObjectName("idle");
    idle_->assignProperty(start_reset_button_, "text", "Reset");
    idle_->assignProperty(start_reset_button_, "toolTip", shutdown_msg);
    idle_->assignProperty(start_reset_button_, "enabled", true);

    idle_->assignProperty(pause_resume_button_, "text", "Pause");
    idle_->assignProperty(pause_resume_button_, "enabled", true);
    idle_->assignProperty(pause_resume_button_, "toolTip", pause_msg);

    idle_->assignProperty(navigation_mode_button_, "text", "Waypoint / Nav Through Poses Mode");
    idle_->assignProperty(navigation_mode_button_, "enabled", true);
    idle_->assignProperty(navigation_mode_button_, "toolTip", single_goal_msg);

    // State entered when navigate_to_pose action is not active
    accumulating_ = new QState();
    accumulating_->setObjectName("accumulating");
    accumulating_->assignProperty(start_reset_button_, "text", "Cancel Accumulation");
    accumulating_->assignProperty(start_reset_button_, "toolTip", cancel_waypoint_msg);
    accumulating_->assignProperty(start_reset_button_, "enabled", true);

    accumulating_->assignProperty(pause_resume_button_, "text", "Start Nav Through Poses");
    accumulating_->assignProperty(pause_resume_button_, "enabled", true);
    accumulating_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

    accumulating_->assignProperty(navigation_mode_button_, "text", "Start Waypoint Following");
    accumulating_->assignProperty(navigation_mode_button_, "enabled", true);
    accumulating_->assignProperty(navigation_mode_button_, "toolTip", waypoint_goal_msg);

    accumulated_wp_ = new QState();
    accumulated_wp_->setObjectName("accumulated_wp");
    accumulated_wp_->assignProperty(start_reset_button_, "text", "Cancel");
    accumulated_wp_->assignProperty(start_reset_button_, "toolTip", cancel_msg);
    accumulated_wp_->assignProperty(start_reset_button_, "enabled", true);

    accumulated_wp_->assignProperty(pause_resume_button_, "text", "Start Nav Through Poses");
    accumulated_wp_->assignProperty(pause_resume_button_, "enabled", false);
    accumulated_wp_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

    accumulated_wp_->assignProperty(navigation_mode_button_, "text", "Start Waypoint Following");
    accumulated_wp_->assignProperty(navigation_mode_button_, "enabled", false);
    accumulated_wp_->assignProperty(navigation_mode_button_, "toolTip", waypoint_goal_msg);

    accumulated_nav_through_poses_ = new QState();
    accumulated_nav_through_poses_->setObjectName("accumulated_nav_through_poses");
    accumulated_nav_through_poses_->assignProperty(start_reset_button_, "text", "Cancel");
    accumulated_nav_through_poses_->assignProperty(start_reset_button_, "toolTip", cancel_msg);
    accumulated_nav_through_poses_->assignProperty(start_reset_button_, "enabled", true);

    accumulated_nav_through_poses_->assignProperty(
        pause_resume_button_, "text",
        "Start Nav Through Poses");
    accumulated_nav_through_poses_->assignProperty(pause_resume_button_, "enabled", false);
    accumulated_nav_through_poses_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

    accumulated_nav_through_poses_->assignProperty(
        navigation_mode_button_, "text",
        "Start Waypoint Following");
    accumulated_nav_through_poses_->assignProperty(navigation_mode_button_, "enabled", false);
    accumulated_nav_through_poses_->assignProperty(
        navigation_mode_button_, "toolTip",
        waypoint_goal_msg);

    // State entered to cancel the navigate_to_pose action
    canceled_ = new QState();
    canceled_->setObjectName("canceled");

    // State entered to reset the nav2 lifecycle nodes
    reset_ = new QState();
    reset_->setObjectName("reset");

    // State entered while the navigate_to_pose action is active
    running_ = new QState();
    running_->setObjectName("running");
    running_->assignProperty(start_reset_button_, "text", "Cancel");
    running_->assignProperty(start_reset_button_, "toolTip", cancel_msg);

    running_->assignProperty(pause_resume_button_, "text", "Pause");
    running_->assignProperty(pause_resume_button_, "enabled", false);

    running_->assignProperty(navigation_mode_button_, "text", "Waypoint mode");
    running_->assignProperty(navigation_mode_button_, "enabled", false);

    // State entered when pause is requested
    paused_ = new QState();
    paused_->setObjectName("pausing");
    paused_->assignProperty(start_reset_button_, "text", "Reset");
    paused_->assignProperty(start_reset_button_, "toolTip", shutdown_msg);

    paused_->assignProperty(pause_resume_button_, "text", "Resume");
    paused_->assignProperty(pause_resume_button_, "toolTip", resume_msg);
    paused_->assignProperty(pause_resume_button_, "enabled", true);

    paused_->assignProperty(navigation_mode_button_, "text", "Start navigation");
    paused_->assignProperty(navigation_mode_button_, "toolTip", resume_msg);
    paused_->assignProperty(navigation_mode_button_, "enabled", true);

    // State entered to resume the nav2 lifecycle nodes
    resumed_ = new QState();
    resumed_->setObjectName("resuming");

    QObject::connect(initial_, SIGNAL(exited()), this, SLOT(onStartup()));
    QObject::connect(canceled_, SIGNAL(exited()), this, SLOT(onCancel()));
    QObject::connect(reset_, SIGNAL(exited()), this, SLOT(onShutdown()));
    QObject::connect(paused_, SIGNAL(entered()), this, SLOT(onPause()));
    QObject::connect(resumed_, SIGNAL(exited()), this, SLOT(onResume()));
    QObject::connect(accumulating_, SIGNAL(entered()), this, SLOT(onAccumulating()));
    QObject::connect(accumulated_wp_, SIGNAL(entered()), this, SLOT(onAccumulatedWp()));
    QObject::connect(accumulated_nav_through_poses_, SIGNAL(entered()), this, SLOT(onAccumulatedNTP()));

    // Start/Reset button click transitions
    initial_->addTransition(start_reset_button_, SIGNAL(clicked()), idle_);
    idle_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);
    running_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
    paused_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);
    idle_->addTransition(navigation_mode_button_, SIGNAL(clicked()), accumulating_);
    accumulating_->addTransition(navigation_mode_button_, SIGNAL(clicked()), accumulated_wp_);
    accumulating_->addTransition(
        pause_resume_button_, SIGNAL(
            clicked()), accumulated_nav_through_poses_);
    accumulating_->addTransition(start_reset_button_, SIGNAL(clicked()), idle_);
    accumulated_wp_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
    accumulated_nav_through_poses_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);

    // Internal state transitions
    canceled_->addTransition(canceled_, SIGNAL(entered()), idle_);
    reset_->addTransition(reset_, SIGNAL(entered()), initial_);
    resumed_->addTransition(resumed_, SIGNAL(entered()), idle_);

    // Pause/Resume button click transitions
    idle_->addTransition(pause_resume_button_, SIGNAL(clicked()), paused_);
    paused_->addTransition(pause_resume_button_, SIGNAL(clicked()), resumed_);

    state_machine_.addState(pre_initial_);
    state_machine_.addState(initial_);
    state_machine_.addState(idle_);
    state_machine_.addState(running_);
    state_machine_.addState(canceled_);
    state_machine_.addState(reset_);
    state_machine_.addState(paused_);
    state_machine_.addState(resumed_);
    state_machine_.addState(accumulating_);
    state_machine_.addState(accumulated_wp_);
    state_machine_.addState(accumulated_nav_through_poses_);

    state_machine_.setInitialState(pre_initial_);

    // delay starting initial thread until state machine has started or a race occurs
    QObject::connect(&state_machine_, SIGNAL(started()), this, SLOT(startThread()));
    state_machine_.start();
    }

    // Lay out the items in the panel
    QVBoxLayout * main_layout = new QVBoxLayout;
    // main_layout->addWidget(navigation_status_indicator_);
    // main_layout->addWidget(localization_status_indicator_);
    // main_layout->addWidget(navigation_goal_status_indicator_);
    // main_layout->addWidget(navigation_feedback_indicator_);
    // main_layout->addWidget(pause_resume_button_);
    // main_layout->addWidget(start_reset_button_);
    // main_layout->addWidget(navigation_mode_button_);

    // main_layout->addWidget(lyt_buttons);
    main_layout->insertLayout(0, lyt_buttons);

    main_layout->setContentsMargins(10, 10, 10, 10);
    setLayout(main_layout);

}

RbqPanel::~RbqPanel()
{
    // Stop executor and join thread - SAFER IMPLEMENTATION
    try {
        if (executor_) {
            executor_->cancel();
        }
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    } catch (const std::exception& e) {
        // Log error but don't crash
        std::cerr << "Error in RbqPanel destructor: " << e.what() << std::endl;
    }
}

void RbqPanel::onInitialize()
{

}

void RbqPanel::startThread()
{

}

void RbqPanel::onPause()
{

}

void RbqPanel::onResume()
{

}

void RbqPanel::onStartup()
{

}

void RbqPanel::onShutdown()
{
    timer_.stop();
}

void RbqPanel::onCancel()
{
    QFuture<void> future =
        QtConcurrent::run(
            std::bind(
                &RbqPanel::onCancelButtonPressed,
                this));
}

void RbqPanel::onNewGoal(double x, double y, double theta, QString frame)
{
    updateWpNavigationMarkers();
    Q_UNUSED(x);
    Q_UNUSED(y);
    Q_UNUSED(theta);
    Q_UNUSED(frame);
}



void RbqPanel::autoStart         () { publisher->pub_autoStart         (); }
void RbqPanel::canCheck          () { publisher->pub_canCheck          (); }
void RbqPanel::findHome          () { publisher->pub_findHome          (); }
void RbqPanel::sit               () { publisher->pub_switchGait(0); }
void RbqPanel::stand             () { publisher->pub_switchGait(1); }
void RbqPanel::walk              () { publisher->pub_switchGait(2); }
void RbqPanel::walkSlow          () { publisher->pub_switchGait(5); }
void RbqPanel::run               () { publisher->pub_switchGait(6); }
void RbqPanel::dock              () { publisher->pub_dock              (); }

// RL 관련 함수들 구현
void RbqPanel::stairs            () { publisher->pub_switchGait(4); }
void RbqPanel::rlTrot            () { publisher->pub_switchGait(30); }
void RbqPanel::rlBound           () { publisher->pub_switchGait(35); }
void RbqPanel::rlPace            () { publisher->pub_switchGait(36); }
void RbqPanel::rlPronk           () { publisher->pub_switchGait(37); }
void RbqPanel::rlTrotVision      () { publisher->pub_switchGait(42); }
void RbqPanel::rlTrotRun         () { publisher->pub_switchGait(45); }
void RbqPanel::rlSilent          () { publisher->pub_switchGait(46); }
void RbqPanel::rlFrontWalk       () { publisher->pub_switchGait(31); }
void RbqPanel::emergency         () { publisher->pub_emergency         (); }

void RbqPanel::extJoyToggle()
{
    // 현재 상태를 반대로 토글
    highLevel_state_ = !highLevel_state_;
    
    qDebug() << "Control Mode Toggle - New state:" << highLevel_state_ << "Publishing:" << highLevel_state_;
    
    // 새로운 상태로 메시지 발행
    publisher->pub_switchControlMode(highLevel_state_);
    
    // 버튼 상태와 색상 업데이트
    bt_extJoy->setChecked(highLevel_state_);
    
    if (highLevel_state_) {
        // 활성화된 경우 - 녹색
        bt_extJoy->setStyleSheet("QPushButton { background-color: #90EE90; color: #2d5016; font-weight: bold; border: 2px solid #7CCD7C; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #7CCD7C; } QPushButton:pressed { background-color: #6BB86B; }");
    } else {
        // 비활성화된 경우 - 회색
        bt_extJoy->setStyleSheet("QPushButton { background-color: #f0f0f0; color: #333333; font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #e0e0e0; } QPushButton:pressed { background-color: #d0d0d0; }");
    }
}
void RbqPanel::powerLeg          (const bool &powerON) { publisher->pub_setPortState(0x00, powerON); }
void RbqPanel::powerArm          (const bool &powerON) { publisher->pub_setPortState(0x01, powerON); }
void RbqPanel::powerVisionPC     (const bool &powerON) { publisher->pub_setPortState(0x10, powerON); }
void RbqPanel::powerUsbHub       (const bool &powerON) { publisher->pub_setPortState(0x21, powerON); }
void RbqPanel::powerCctv         (const bool &powerON) { publisher->pub_setPortState(0x13, powerON); }
void RbqPanel::powerThermal      (const bool &powerON) { publisher->pub_setPortState(0x14, powerON); }
void RbqPanel::powerLidar        (const bool &powerON) { publisher->pub_setPortState(0x12, powerON); }
void RbqPanel::powerExt52V       (const bool &powerON) { publisher->pub_setPortState(0x02, powerON); }
void RbqPanel::powerIrLEDs       (const bool &powerON) { publisher->pub_setPortState(0x15, powerON); }
void RbqPanel::powerComm         (const bool &powerON) { publisher->pub_setPortState(0x11, powerON); }
void RbqPanel::setBodyHeight     (const int  &height ) { publisher->pub_setBodyHeight     (height ); }
void RbqPanel::setFootHeight     (const int  &height ) { publisher->pub_setFootHeight     (height ); }
void RbqPanel::setMaxSpeed       (const int  &speed  ) { publisher->pub_setMaxSpeed       (speed  ); }
void RbqPanel::comEstimation     (const int  &stage  ) { publisher->pub_comEstimation     (stage  ); }

// Status callback functions - THREAD SAFE IMPLEMENTATION
void RbqPanel::statusCallback(const rbq_msgs::msg::RobotStatus::SharedPtr msg)
{
    // Store data for batch update (thread-safe)
    std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
    robot_data_.robot_status = msg;
}

void RbqPanel::batteryStateCallback(const rbq_msgs::msg::BatteryState::SharedPtr msg)
{
    // Store data for batch update (thread-safe)
    std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
    robot_data_.battery_state = msg;
}

void RbqPanel::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Store data for batch update (thread-safe)
    std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
    robot_data_.joint_states = msg;
}

void RbqPanel::jointStatusCallback(const rbq_msgs::msg::JointStatus::SharedPtr msg)
{
    // Store data for batch update (thread-safe)
    std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
    robot_data_.joint_status = msg;
}

void RbqPanel::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Store data for batch update (thread-safe)
    std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
    robot_data_.odometry = msg;
}

void RbqPanel::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Store data for batch update (thread-safe)
    std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
    robot_data_.imu = msg;
}

void RbqPanel::executorThreadFunction()
{
    executor_->spin();
}

void RbqPanel::batchUpdateTables()
{
    // Lock data for reading
    std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
    
    // Update status indicators if robot_status is available
    if (robot_data_.robot_status) {
        auto msg = robot_data_.robot_status;
        if (lbl_motor_check_status || lbl_initialize_pose_status || 
            lbl_control_start_status || lbl_imu_status) {
            
            // Update status indicators based on robot status - RECTANGULAR STYLE
            if (lbl_motor_check_status) {
                if (msg->can_check) {
                    lbl_motor_check_status->setStyleSheet("QLabel { background-color: green; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                } else {
                    lbl_motor_check_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                }
            }
            
            if (lbl_initialize_pose_status) {
                if (msg->find_home) {
                    lbl_initialize_pose_status->setStyleSheet("QLabel { background-color: green; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                } else {
                    lbl_initialize_pose_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                }
            }
            
            if (lbl_control_start_status) {
                if (msg->con_start) {
                    lbl_control_start_status->setStyleSheet("QLabel { background-color: green; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                } else {
                    lbl_control_start_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                }
            }
            
            if (lbl_imu_status) {
                if (msg->imu_success) {
                    lbl_imu_status->setStyleSheet("QLabel { background-color: green; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                } else {
                    lbl_imu_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                }
            }
            
            // Update gait button colors based on current gait
            updateGaitButtonColors(msg->gait_id);
            
            // Update ext joy button color based on ext_joy status
            updateExtJoyButtonColor(msg->ext_joy);
        }
    }
    
    // Update battery voltage if battery_state is available
    if (robot_data_.battery_state && lbl_battery_voltage) {
        auto msg = robot_data_.battery_state;
        double voltage = msg->voltage;
        QString voltage_text = QString::number(voltage, 'f', 1) + " V";
        lbl_battery_voltage->setText(voltage_text);
    }
    
    // Update joint tables if data is available
    if (robot_data_.joint_status && tab_widget_) {
        auto msg = robot_data_.joint_status;
        QWidget* advanced_tab = tab_widget_->widget(1); // State 탭
        if (advanced_tab) {
            QGroupBox* joint_group = advanced_tab->findChild<QGroupBox*>("Joint States");
            if (joint_group) {
                QTableWidget* joint_table = joint_group->findChild<QTableWidget*>();
                if (joint_table) {
                    // 테이블 업데이트 비활성화 (배치 업데이트를 위해)
                    joint_table->setUpdatesEnabled(false);
                    
                    // JointStatus 메시지에서 데이터 추출하여 테이블 업데이트
                    for (size_t i = 0; i < msg->connected.size() && i < 16; ++i) {
                        // Status (연결 상태)
                        QTableWidgetItem* status_item = joint_table->item(i, 0);
                        if (status_item) {
                            if (msg->connected[i]) {
                                status_item->setText("Y/0/0/0"); // 연결됨
                                status_item->setBackground(QColor(200, 255, 200)); // 연한 녹색
                            } else {
                                status_item->setText("N/-/-/0"); // 연결 안됨
                                status_item->setBackground(QColor(255, 200, 200)); // 연한 빨간색
                            }
                        }
                        
                        // Error (status bits에서 추출)
                        QTableWidgetItem* error_item = joint_table->item(i, 1);
                        if (error_item) {
                            int error_count = 0;
                            if (i < msg->status_flt.size() && msg->status_flt[i]) error_count++;
                            if (i < msg->status_tmp.size() && msg->status_tmp[i]) error_count++;
                            error_item->setText(QString::number(error_count) + "/0");
                        }
                        
                        // Temperature
                        QTableWidgetItem* temp_item = joint_table->item(i, 2);
                        if (temp_item && i < msg->temperature.size()) {
                            temp_item->setText(QString::number(msg->temperature[i]) + "/" + QString::number(msg->motor_temp[i]));
                        }
                        
                        // Angle Ref
                        QTableWidgetItem* angle_ref_item = joint_table->item(i, 3);
                        if (angle_ref_item && i < msg->position_ref.size()) {
                            angle_ref_item->setText(QString::number(msg->position_ref[i], 'f', 3));
                        }
                        
                        // Angle Enc
                        QTableWidgetItem* angle_enc_item = joint_table->item(i, 4);
                        if (angle_enc_item && i < msg->position_enc.size()) {
                            angle_enc_item->setText(QString::number(msg->position_enc[i], 'f', 3));
                        }
                        
                        // Torque Ref
                        QTableWidgetItem* torque_ref_item = joint_table->item(i, 5);
                        if (torque_ref_item && i < msg->torque_ref.size()) {
                            torque_ref_item->setText(QString::number(msg->torque_ref[i], 'f', 3));
                        }
                        
                        // Current Sen
                        QTableWidgetItem* current_item = joint_table->item(i, 6);
                        if (current_item && i < msg->current.size()) {
                            current_item->setText(QString::number(msg->current[i], 'f', 3));
                        }
                        
                        // KP
                        QTableWidgetItem* kp_item = joint_table->item(i, 7);
                        if (kp_item && i < msg->kp.size()) {
                            kp_item->setText(QString::number(msg->kp[i], 'f', 3));
                        }
                        
                        // KD
                        QTableWidgetItem* kd_item = joint_table->item(i, 8);
                        if (kd_item && i < msg->kd.size()) {
                            kd_item->setText(QString::number(msg->kd[i], 'f', 3));
                        }
                        
                        // Owner
                        QTableWidgetItem* owner_item = joint_table->item(i, 9);
                        if (owner_item && i < msg->owner.size()) {
                            owner_item->setText(QString::number(msg->owner[i]));
                        }
                    }
                    
                    // 테이블 업데이트 재활성화 및 한 번에 리페인트
                    joint_table->setUpdatesEnabled(true);
                    joint_table->update(); // 전체 테이블을 한 번에 업데이트
                }
            }
        }
    }
    
    // Update odometry tables if data is available
    if (robot_data_.odometry && tab_widget_) {
        auto msg = robot_data_.odometry;
        QWidget* advanced_tab = tab_widget_->widget(1); // State 탭
        if (advanced_tab) {
            // Robot Pose GroupBox에서 position_table과 orientation_table을 찾기
            QGroupBox* robot_pose_group = advanced_tab->findChild<QGroupBox*>("Robot Pose");
            if (robot_pose_group) {
                QList<QTableWidget*> pose_tables = robot_pose_group->findChildren<QTableWidget*>();
                if (pose_tables.size() >= 2) {
                    QTableWidget* position_table = pose_tables[0]; // 첫 번째 테이블이 position_table
                    QTableWidget* orientation_table = pose_tables[1]; // 두 번째 테이블이 orientation_table
                    
                    // 테이블 업데이트 비활성화 (배치 업데이트를 위해)
                    position_table->setUpdatesEnabled(false);
                    orientation_table->setUpdatesEnabled(false);
                    
                    // odometry 메시지에서 데이터 추출하여 테이블 업데이트
                    // Position (X, Y, Z)
                    QTableWidgetItem* px_item = position_table->item(0, 0);
                    if (px_item) {
                        px_item->setText(QString::number(msg->pose.pose.position.x, 'f', 3));
                    }
                    
                    QTableWidgetItem* py_item = position_table->item(0, 1);
                    if (py_item) {
                        py_item->setText(QString::number(msg->pose.pose.position.y, 'f', 3));
                    }
                    
                    QTableWidgetItem* pz_item = position_table->item(0, 2);
                    if (pz_item) {
                        pz_item->setText(QString::number(msg->pose.pose.position.z, 'f', 3));
                    }
                    
                    // Orientation (Quaternion X, Y, Z, W)
                    QTableWidgetItem* ox_item = orientation_table->item(0, 0);
                    if (ox_item) {
                        ox_item->setText(QString::number(msg->pose.pose.orientation.x, 'f', 3));
                    }
                    
                    QTableWidgetItem* oy_item = orientation_table->item(0, 1);
                    if (oy_item) {
                        oy_item->setText(QString::number(msg->pose.pose.orientation.y, 'f', 3));
                    }
                    
                    QTableWidgetItem* oz_item = orientation_table->item(0, 2);
                    if (oz_item) {
                        oz_item->setText(QString::number(msg->pose.pose.orientation.z, 'f', 3));
                    }
                    
                    QTableWidgetItem* ow_item = orientation_table->item(0, 3);
                    if (ow_item) {
                        ow_item->setText(QString::number(msg->pose.pose.orientation.w, 'f', 3));
                    }
                    
                    // 테이블 업데이트 재활성화 및 한 번에 리페인트
                    position_table->setUpdatesEnabled(true);
                    orientation_table->setUpdatesEnabled(true);
                    position_table->update();
                    orientation_table->update();
                }
            }
        }
    }
    
    // Update IMU tables if data is available
    if (robot_data_.imu && tab_widget_) {
        auto msg = robot_data_.imu;
        QWidget* advanced_tab = tab_widget_->widget(1); // State 탭
        if (advanced_tab) {
            // IMU Data GroupBox에서 imu_table을 찾기
            QGroupBox* imu_group = advanced_tab->findChild<QGroupBox*>("IMU Data");
            if (imu_group) {
                QTableWidget* imu_table = imu_group->findChild<QTableWidget*>();
                if (imu_table) {
                    // 테이블 업데이트 비활성화 (배치 업데이트를 위해)
                    imu_table->setUpdatesEnabled(false);
                    
                    // IMU 메시지에서 데이터 추출하여 테이블 업데이트
                    // Orientation (Quaternion을 Euler angles로 변환)
                    // Roll (X-axis)
                    QTableWidgetItem* roll_angle_item = imu_table->item(0, 0);
                    if (roll_angle_item) {
                        // Quaternion에서 Roll 계산 (간단한 근사)
                        double roll = atan2(2.0 * (msg->orientation.w * msg->orientation.x + msg->orientation.y * msg->orientation.z),
                                           1.0 - 2.0 * (msg->orientation.x * msg->orientation.x + msg->orientation.y * msg->orientation.y));
                        roll_angle_item->setText(QString::number(roll, 'f', 3));
                    }
                    
                    // Pitch (Y-axis)
                    QTableWidgetItem* pitch_angle_item = imu_table->item(1, 0);
                    if (pitch_angle_item) {
                        double pitch = asin(2.0 * (msg->orientation.w * msg->orientation.y - msg->orientation.z * msg->orientation.x));
                        pitch_angle_item->setText(QString::number(pitch, 'f', 3));
                    }
                    
                    // Yaw (Z-axis)
                    QTableWidgetItem* yaw_angle_item = imu_table->item(2, 0);
                    if (yaw_angle_item) {
                        double yaw = atan2(2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y),
                                          1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z));
                        yaw_angle_item->setText(QString::number(yaw, 'f', 3));
                    }
                    
                    // Angular velocity (Gyro)
                    QTableWidgetItem* roll_gyro_item = imu_table->item(0, 1);
                    if (roll_gyro_item) {
                        roll_gyro_item->setText(QString::number(msg->angular_velocity.x, 'f', 3));
                    }
                    
                    QTableWidgetItem* pitch_gyro_item = imu_table->item(1, 1);
                    if (pitch_gyro_item) {
                        pitch_gyro_item->setText(QString::number(msg->angular_velocity.y, 'f', 3));
                    }
                    
                    QTableWidgetItem* yaw_gyro_item = imu_table->item(2, 1);
                    if (yaw_gyro_item) {
                        yaw_gyro_item->setText(QString::number(msg->angular_velocity.z, 'f', 3));
                    }
                    
                    // Linear acceleration
                    QTableWidgetItem* roll_acc_item = imu_table->item(0, 2);
                    if (roll_acc_item) {
                        roll_acc_item->setText(QString::number(msg->linear_acceleration.x, 'f', 3));
                    }
                    
                    QTableWidgetItem* pitch_acc_item = imu_table->item(1, 2);
                    if (pitch_acc_item) {
                        pitch_acc_item->setText(QString::number(msg->linear_acceleration.y, 'f', 3));
                    }
                    
                    QTableWidgetItem* yaw_acc_item = imu_table->item(2, 2);
                    if (yaw_acc_item) {
                        yaw_acc_item->setText(QString::number(msg->linear_acceleration.z, 'f', 3));
                    }
                    
                    // 테이블 업데이트 재활성화 및 한 번에 리페인트
                    imu_table->setUpdatesEnabled(true);
                    imu_table->update();
                }
            }
        }
    }
}

void RbqPanel::updateGaitButtonColors(int currentGaitId)
{
    // 기본 스타일 (회색)
    QString defaultStyle = "QPushButton { background-color: #f0f0f0; color: #333333; font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #e0e0e0; } QPushButton:pressed { background-color: #d0d0d0; }";
    
    // 활성화된 스타일 (연두색)
    QString activeStyle = "QPushButton { background-color: #90EE90; color: #2d5016; font-weight: bold; border: 2px solid #7CCD7C; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #7CCD7C; } QPushButton:pressed { background-color: #6BB86B; }";
    
    // 모든 gait 버튼을 기본 색상으로 초기화
    bt_sit->setStyleSheet(defaultStyle);
    bt_stand->setStyleSheet(defaultStyle);
    bt_walk->setStyleSheet(defaultStyle);
    bt_walkSlow->setStyleSheet(defaultStyle);
    bt_run->setStyleSheet(defaultStyle);
    
    bt_stairs->setStyleSheet(defaultStyle);
    bt_rlTrot->setStyleSheet(defaultStyle);
    bt_rlBound->setStyleSheet(defaultStyle);
    bt_rlPace->setStyleSheet(defaultStyle);
    bt_rlPronk->setStyleSheet(defaultStyle);
    bt_rlTrotVision->setStyleSheet(defaultStyle);
    bt_rlTrotRun->setStyleSheet(defaultStyle);
    bt_rlSilent->setStyleSheet(defaultStyle);
    bt_rlFrontWalk->setStyleSheet(defaultStyle);
    
    // 현재 gait에 따라 해당 버튼만 연두색으로 설정
    // 정확한 GAIT_STATE enum 값으로 매핑
    switch(currentGaitId) {
        case 0: // SITTING
            bt_sit->setStyleSheet(activeStyle);
            break;
        case 1: // STANDING
            bt_stand->setStyleSheet(activeStyle);
            break;
        case 3: // TROTTING (Walk)
            bt_walk->setStyleSheet(activeStyle);
            break;
        case 5: // WAVING (Wave)
            bt_walkSlow->setStyleSheet(activeStyle);
            break;
        case 6: // TROT_RUNNING (Run)
            bt_run->setStyleSheet(activeStyle);
            break;
        case 4: // TROT_STAIRS (Stairs)
            bt_stairs->setStyleSheet(activeStyle);
            break;
        case 30: // RL_TROT
            bt_rlTrot->setStyleSheet(activeStyle);
            break;
        case 31: // RL_FRONT_WALK
            bt_rlFrontWalk->setStyleSheet(activeStyle);
            break;
        case 35: // RL_BOUND
            bt_rlBound->setStyleSheet(activeStyle);
            break;
        case 36: // RL_PACE
            bt_rlPace->setStyleSheet(activeStyle);
            break;
        case 37: // RL_PRONK
            bt_rlPronk->setStyleSheet(activeStyle);
            break;
        case 42: // RL_TROT_VISION
            bt_rlTrotVision->setStyleSheet(activeStyle);
            break;
        case 45: // RL_TROT_RUN
            bt_rlTrotRun->setStyleSheet(activeStyle);
            break;
        case 46: // RL_SILENT
            bt_rlSilent->setStyleSheet(activeStyle);
            break;
        case 47: // RL_STAIRS
            bt_stairs->setStyleSheet(activeStyle);
            break;
        default:
            // 알 수 없는 gait_id인 경우 모든 버튼을 기본 색상으로 유지
            break;
    }
}

void RbqPanel::updateExtJoyButtonColor(bool extJoyConnected)
{
    if (!bt_extJoy) return;
    
    // 외부 상태에 따라 내부 상태 업데이트
    highLevel_state_ = extJoyConnected;
    
    // 토픽에서 받은 상태에 따라 버튼 색상 업데이트
    if (extJoyConnected) {
        // Ext joy가 연결된 경우 - 녹색
        bt_extJoy->setStyleSheet("QPushButton { background-color: #90EE90; color: #2d5016; font-weight: bold; border: 2px solid #7CCD7C; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #7CCD7C; } QPushButton:pressed { background-color: #6BB86B; }");
        bt_extJoy->setChecked(true);
    } else {
        // Ext joy가 연결되지 않은 경우 - 회색
        bt_extJoy->setStyleSheet("QPushButton { background-color: #f0f0f0; color: #333333; font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #e0e0e0; } QPushButton:pressed { background-color: #d0d0d0; }");
        bt_extJoy->setChecked(false);
    }
}

void RbqPanel::updateJointTable()
{
    // ROS2 토픽에서 joint 데이터를 받아와서 테이블 업데이트
    // 실제 구현에서는 joint_states 토픽을 구독하여 데이터를 받아와야 함
    // 현재는 예시 데이터로 구현
    
    if (!tab_widget_) return;
    
    QWidget* advanced_tab = tab_widget_->widget(1); // Advanced 탭
    if (!advanced_tab) return;
    
    QTableWidget* joint_table = advanced_tab->findChild<QTableWidget*>();
    if (!joint_table) return;
    
    // 예시 데이터 (실제로는 ROS2 토픽에서 받아온 데이터 사용)
    for (int row = 0; row < 16; ++row) {
        // Status (연결 상태에 따라 색상 변경)
        QTableWidgetItem* status_item = joint_table->item(row, 0);
        if (status_item) {
            status_item->setText("Y/0/0/0"); // 연결됨
            status_item->setBackground(QColor(200, 255, 200)); // 연한 녹색
        }
        
        // Error
        QTableWidgetItem* error_item = joint_table->item(row, 1);
        if (error_item) {
            error_item->setText("0/0");
        }
        
        // Temperature
        QTableWidgetItem* temp_item = joint_table->item(row, 2);
        if (temp_item) {
            temp_item->setText(QString::number(25.0 + row, 'f', 1));
        }
        
        // Angle (position)
        QTableWidgetItem* angle_item = joint_table->item(row, 3);
        if (angle_item) {
            angle_item->setText(QString::number(0.1 * row, 'f', 3));
        }
        
        // Torque
        QTableWidgetItem* torque_item = joint_table->item(row, 4);
        if (torque_item) {
            torque_item->setText(QString::number(0.05 * row, 'f', 3));
        }
        
        // Current
        QTableWidgetItem* current_item = joint_table->item(row, 5);
        if (current_item) {
            current_item->setText(QString::number(0.02 * row, 'f', 3));
        }
        
        // KP
        QTableWidgetItem* kp_item = joint_table->item(row, 6);
        if (kp_item) {
            kp_item->setText(QString::number(10.0 + row, 'f', 1));
        }
        
        // KD
        QTableWidgetItem* kd_item = joint_table->item(row, 7);
        if (kd_item) {
            kd_item->setText(QString::number(1.0 + 0.1 * row, 'f', 1));
        }
        
        // Owner
        QTableWidgetItem* owner_item = joint_table->item(row, 8);
        if (owner_item) {
            owner_item->setText("0");
        }
    }
}

void RbqPanel::updateOdometryTable()
{
    // ROS2 토픽에서 odometry 데이터를 받아와서 테이블 업데이트
    // 실제 구현에서는 odometry 토픽을 구독하여 데이터를 받아와야 함
    // 현재는 예시 데이터로 구현
    
    if (!tab_widget_) return;
    
    QWidget* advanced_tab = tab_widget_->widget(1); // Advanced 탭
    if (!advanced_tab) return;
    
    // Robot Pose GroupBox에서 position_table과 orientation_table을 찾기
    QGroupBox* robot_pose_group = advanced_tab->findChild<QGroupBox*>("Robot Pose");
    if (!robot_pose_group) return;
    
    QList<QTableWidget*> pose_tables = robot_pose_group->findChildren<QTableWidget*>();
    if (pose_tables.size() < 2) return;
    
    QTableWidget* position_table = pose_tables[0]; // 첫 번째 테이블이 position_table
    QTableWidget* orientation_table = pose_tables[1]; // 두 번째 테이블이 orientation_table
    
    // 예시 데이터 (실제로는 ROS2 odometry 토픽에서 받아온 데이터 사용)
    // Position (X, Y, Z)
    QTableWidgetItem* px_item = position_table->item(0, 0);
    if (px_item) {
        px_item->setText(QString::number(1.234, 'f', 3));
    }
    
    QTableWidgetItem* py_item = position_table->item(0, 1);
    if (py_item) {
        py_item->setText(QString::number(2.345, 'f', 3));
    }
    
    QTableWidgetItem* pz_item = position_table->item(0, 2);
    if (pz_item) {
        pz_item->setText(QString::number(0.123, 'f', 3));
    }
    
    // Orientation (Quaternion X, Y, Z, W)
    QTableWidgetItem* ox_item = orientation_table->item(0, 0);
    if (ox_item) {
        ox_item->setText(QString::number(0.0, 'f', 3));
    }
    
    QTableWidgetItem* oy_item = orientation_table->item(0, 1);
    if (oy_item) {
        oy_item->setText(QString::number(0.0, 'f', 3));
    }
    
    QTableWidgetItem* oz_item = orientation_table->item(0, 2);
    if (oz_item) {
        oz_item->setText(QString::number(0.707, 'f', 3));
    }
    
    QTableWidgetItem* ow_item = orientation_table->item(0, 3);
    if (ow_item) {
        ow_item->setText(QString::number(0.707, 'f', 3));
    }
}

void RbqPanel::updateImuTable()
{
    // ROS2 토픽에서 IMU 데이터를 받아와서 테이블 업데이트
    // 실제 구현에서는 IMU 토픽을 구독하여 데이터를 받아와야 함
    // 현재는 예시 데이터로 구현
    
    if (!tab_widget_) return;
    
    QWidget* advanced_tab = tab_widget_->widget(1); // State 탭
    if (!advanced_tab) return;
    
    // IMU Data GroupBox에서 imu_table을 찾기
    QGroupBox* imu_group = advanced_tab->findChild<QGroupBox*>("IMU Data");
    if (!imu_group) return;
    QTableWidget* imu_table = imu_group->findChild<QTableWidget*>();
    if (!imu_table) return;
    
    // 예시 데이터 (실제로는 ROS2 IMU 토픽에서 받아온 데이터 사용)
    // Angle (Roll, Pitch, Yaw)
    QTableWidgetItem* roll_angle_item = imu_table->item(0, 0);
    if (roll_angle_item) {
        roll_angle_item->setText(QString::number(0.123, 'f', 3));
    }
    
    QTableWidgetItem* pitch_angle_item = imu_table->item(1, 0);
    if (pitch_angle_item) {
        pitch_angle_item->setText(QString::number(-0.045, 'f', 3));
    }
    
    QTableWidgetItem* yaw_angle_item = imu_table->item(2, 0);
    if (yaw_angle_item) {
        yaw_angle_item->setText(QString::number(1.567, 'f', 3));
    }
    
    // Gyro (Angular velocity)
    QTableWidgetItem* roll_gyro_item = imu_table->item(0, 1);
    if (roll_gyro_item) {
        roll_gyro_item->setText(QString::number(0.012, 'f', 3));
    }
    
    QTableWidgetItem* pitch_gyro_item = imu_table->item(1, 1);
    if (pitch_gyro_item) {
        pitch_gyro_item->setText(QString::number(-0.008, 'f', 3));
    }
    
    QTableWidgetItem* yaw_gyro_item = imu_table->item(2, 1);
    if (yaw_gyro_item) {
        yaw_gyro_item->setText(QString::number(0.003, 'f', 3));
    }
    
    // Acc (Linear acceleration)
    QTableWidgetItem* roll_acc_item = imu_table->item(0, 2);
    if (roll_acc_item) {
        roll_acc_item->setText(QString::number(0.001, 'f', 3));
    }
    
    QTableWidgetItem* pitch_acc_item = imu_table->item(1, 2);
    if (pitch_acc_item) {
        pitch_acc_item->setText(QString::number(0.002, 'f', 3));
    }
    
    QTableWidgetItem* yaw_acc_item = imu_table->item(2, 2);
    if (yaw_acc_item) {
        yaw_acc_item->setText(QString::number(-0.001, 'f', 3));
    }
}

QWidget* RbqPanel::createMainTab()
{
    QWidget* main_tab = new QWidget();
    QVBoxLayout* main_layout = new QVBoxLayout(main_tab);

    // Emergency 버튼을 맨 위에 배치
    QHBoxLayout* lyt_emergency = new QHBoxLayout;
    lyt_emergency->addWidget(bt_emergency);
    main_layout->addLayout(lyt_emergency);

    // Status Display Section
    QGroupBox* grp_status = new QGroupBox("Status");
    grp_status->setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }");
    QHBoxLayout* lyt_status = new QHBoxLayout;
    
    // Motor Check - TEXT FIRST, INDICATOR BELOW
    QVBoxLayout* lyt_motor_check = new QVBoxLayout;
    QLabel* lbl_motor_check_text = new QLabel("Motor\nCheck");
    lbl_motor_check_text->setAlignment(Qt::AlignCenter);
    lyt_motor_check->addWidget(lbl_motor_check_text);
    lyt_motor_check->addWidget(lbl_motor_check_status);
    lyt_status->addLayout(lyt_motor_check);
    
    // Initialize Pose - TEXT FIRST, INDICATOR BELOW
    QVBoxLayout* lyt_init_pose = new QVBoxLayout;
    QLabel* lbl_init_pose_text = new QLabel("Initialize\nPose");
    lbl_init_pose_text->setAlignment(Qt::AlignCenter);
    lyt_init_pose->addWidget(lbl_init_pose_text);
    lyt_init_pose->addWidget(lbl_initialize_pose_status);
    lyt_status->addLayout(lyt_init_pose);
    
    // Control Start - TEXT FIRST, INDICATOR BELOW
    QVBoxLayout* lyt_control_start = new QVBoxLayout;
    QLabel* lbl_control_start_text = new QLabel("Control\nStart");
    lbl_control_start_text->setAlignment(Qt::AlignCenter);
    lyt_control_start->addWidget(lbl_control_start_text);
    lyt_control_start->addWidget(lbl_control_start_status);
    lyt_status->addLayout(lyt_control_start);
    
    // Battery - TEXT FIRST, INDICATOR BELOW
    QVBoxLayout* lyt_battery = new QVBoxLayout;
    QLabel* lbl_battery_text = new QLabel("Battery");
    lbl_battery_text->setAlignment(Qt::AlignCenter);
    lyt_battery->addWidget(lbl_battery_text);
    lyt_battery->addWidget(lbl_battery_voltage);
    lyt_status->addLayout(lyt_battery);
    
    // IMU - TEXT FIRST, INDICATOR BELOW
    QVBoxLayout* lyt_imu = new QVBoxLayout;
    QLabel* lbl_imu_text = new QLabel("IMU");
    lbl_imu_text->setAlignment(Qt::AlignCenter);
    lyt_imu->addWidget(lbl_imu_text);
    lyt_imu->addWidget(lbl_imu_status);
    lyt_status->addLayout(lyt_imu);
    
    grp_status->setLayout(lyt_status);
    main_layout->addWidget(grp_status);

    // Initialize 그룹박스
    QGroupBox* grp_initialize = new QGroupBox("Initialize");
    grp_initialize->setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }");
    QHBoxLayout* lyt_initialize = new QHBoxLayout;
    lyt_initialize->addWidget(bt_autoStart  );
    lyt_initialize->addWidget(bt_canCheck   );
    lyt_initialize->addWidget(bt_findHome   );
    grp_initialize->setLayout(lyt_initialize);
    main_layout->addWidget(grp_initialize);

    // Gait Select 그룹박스
    QGroupBox* grp_gait = new QGroupBox("Gait Select");
    grp_gait->setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }");
    QVBoxLayout* lyt_gait = new QVBoxLayout;
    
    // 첫 번째 행: 기본 보행 버튼들
    QHBoxLayout* lyt_gait_row1 = new QHBoxLayout;
    lyt_gait_row1->addWidget(bt_sit      );
    lyt_gait_row1->addWidget(bt_stand    );
    lyt_gait_row1->addWidget(bt_walk     );
    lyt_gait_row1->addWidget(bt_walkSlow );
    lyt_gait_row1->addWidget(bt_run      );
    lyt_gait_row1->addWidget(bt_stairs       );
    lyt_gait->addLayout(lyt_gait_row1);
    
    QHBoxLayout* lyt_gait_row2 = new QHBoxLayout;
    lyt_gait_row2->addWidget(bt_rlTrot       );
    lyt_gait_row2->addWidget(bt_rlBound      );
    lyt_gait_row2->addWidget(bt_rlPace       );
    lyt_gait_row2->addWidget(bt_rlPronk      );
    lyt_gait->addLayout(lyt_gait_row2);
    
    // 세 번째 행: 추가 RL 보행 버튼들
    QHBoxLayout* lyt_gait_row3 = new QHBoxLayout;
    lyt_gait_row3->addWidget(bt_rlTrotVision );
    lyt_gait_row3->addWidget(bt_rlTrotRun    );
    lyt_gait_row3->addWidget(bt_rlSilent     );
    lyt_gait_row3->addWidget(bt_rlFrontWalk  );
    lyt_gait->addLayout(lyt_gait_row3);
    
    grp_gait->setLayout(lyt_gait);
    main_layout->addWidget(grp_gait);


    // Power 그룹박스 - 3x3 그리드 + 하단 1개 버튼
    QGroupBox* grp_power = new QGroupBox("Power");
    grp_power->setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }");
    QVBoxLayout* lyt_power = new QVBoxLayout;
    
    // 첫 번째 행: Power Leg, Power Arm, Power Ext52V
    QHBoxLayout* lyt_power_row1 = new QHBoxLayout;
    lyt_power_row1->addWidget(bt_powerLeg);
    lyt_power_row1->addWidget(bt_powerArm);
    lyt_power_row1->addWidget(bt_powerExt52V);
    lyt_power->addLayout(lyt_power_row1);
    
    // 두 번째 행: Power VisionPC, Power UsbHub, Power IrLEDs
    QHBoxLayout* lyt_power_row2 = new QHBoxLayout;
    lyt_power_row2->addWidget(bt_powerVisionPC);
    lyt_power_row2->addWidget(bt_powerUsbHub);
    lyt_power_row2->addWidget(bt_powerIrLEDs);
    lyt_power->addLayout(lyt_power_row2);
    
    // 세 번째 행: Power Cctv, Power Thermal, Power Lidar
    QHBoxLayout* lyt_power_row3 = new QHBoxLayout;
    lyt_power_row3->addWidget(bt_powerCctv);
    lyt_power_row3->addWidget(bt_powerThermal);
    lyt_power_row3->addWidget(bt_powerLidar);
    lyt_power->addLayout(lyt_power_row3);
    
    // 네 번째 행: Power Comm (좌우로 꽉 차게)
    QHBoxLayout* lyt_power_row4 = new QHBoxLayout;
    lyt_power_row4->addWidget(bt_powerComm);
    lyt_power->addLayout(lyt_power_row4);
    
    // Power Comm 버튼이 전체 너비를 차지하도록 설정
    bt_powerComm->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    
    grp_power->setLayout(lyt_power);
    main_layout->addWidget(grp_power);

    // Additional Setting 그룹박스
    QGroupBox* grp_additional = new QGroupBox("Additional functions");
    grp_additional->setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }");
    QHBoxLayout* lyt_additional = new QHBoxLayout;
    lyt_additional->addWidget(bt_comEstimation);
    lyt_additional->addWidget(bt_dock);
    lyt_additional->addWidget(bt_extJoy);
    grp_additional->setLayout(lyt_additional);
    main_layout->addWidget(grp_additional);
    
    // Parameter Setting 그룹박스
    QGroupBox* grp_parameter = new QGroupBox("Parameter Setting");
    grp_parameter->setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }");
    QVBoxLayout* lyt_parameter = new QVBoxLayout;
    
    // Height 슬라이더
    QLabel* lbl_height = new QLabel("Body Height");
    QSlider* sldr_setBodyHeight = new QSlider(Qt::Horizontal);
    sldr_setBodyHeight->setTickInterval(10);
    sldr_setBodyHeight->setSliderPosition(50);
    sldr_setBodyHeight->setRange(0, 100);
    sldr_setBodyHeight->setValue(50);
    sldr_setBodyHeight->setSingleStep(10);
    QObject::connect(sldr_setBodyHeight, &QSlider::valueChanged, this, &RbqPanel::setBodyHeight);
    lyt_parameter->addWidget(lbl_height);
    lyt_parameter->addWidget(sldr_setBodyHeight);
    
    // Speed 슬라이더  
    QLabel* lbl_speed = new QLabel("Max Speed");
    QSlider* sldr_setMaxSpeed = new QSlider(Qt::Horizontal);
    sldr_setMaxSpeed->setTickInterval(10);
    sldr_setMaxSpeed->setSliderPosition(50);
    sldr_setMaxSpeed->setRange(0, 100);
    sldr_setMaxSpeed->setValue(50);
    sldr_setMaxSpeed->setSingleStep(10);
    QObject::connect(sldr_setMaxSpeed, &QSlider::valueChanged, this, &RbqPanel::setMaxSpeed);
    lyt_parameter->addWidget(lbl_speed);
    lyt_parameter->addWidget(sldr_setMaxSpeed);
    
    // Foot Height 슬라이더
    QLabel* lbl_foot_height = new QLabel("Foot Height");
    QSlider* sldr_setFootHeight = new QSlider(Qt::Horizontal);
    sldr_setFootHeight->setTickInterval(10);
    sldr_setFootHeight->setSliderPosition(50);
    sldr_setFootHeight->setRange(0, 100);
    sldr_setFootHeight->setValue(50);
    sldr_setFootHeight->setSingleStep(10);
    QObject::connect(sldr_setFootHeight, &QSlider::valueChanged, this, &RbqPanel::setFootHeight);
    lyt_parameter->addWidget(lbl_foot_height);
    lyt_parameter->addWidget(sldr_setFootHeight);
    
    grp_parameter->setLayout(lyt_parameter);
    main_layout->addWidget(grp_parameter);

    return main_tab;
}

QWidget* RbqPanel::createAdvancedTab()
{
    QWidget* advanced_tab = new QWidget();
    QVBoxLayout* advanced_layout = new QVBoxLayout(advanced_tab);
    
    // Joint Table 생성
    QTableWidget* joint_table = new QTableWidget(16, 10);
    joint_table->setHorizontalHeaderLabels({
        "Status", "Error", "Temp", "Angle Ref", "Angle Enc",
        "Torque Ref", "Current Sen", "KP (Nm/rad)", "KD (Nm/rad/s)", "Owner (id)"
    });
    
    // 조인트 이름 설정
    QStringList joint_names = {
        "HRR0", "HRP1", "HRK2", "HLR3", "HLP4", "HLK5",
        "FRR6", "FRP7", "FRK8", "FLR9", "FLP10", "FLK11",
        "HRW12", "HLW13", "FRW14", "FLW15"
    };
    joint_table->setVerticalHeaderLabels(joint_names);
    
    // 테이블 설정
    joint_table->setAlternatingRowColors(true);
    joint_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    joint_table->setSelectionMode(QAbstractItemView::SingleSelection);
    joint_table->setSortingEnabled(false);
    
    // 헤더 설정
    joint_table->horizontalHeader()->setStretchLastSection(true);
    joint_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    joint_table->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    
    // 행 높이 자동 조정
    joint_table->resizeRowsToContents();
    
    // 테이블 크기 설정 - 내용에 맞게 자동 조정
    joint_table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    
    // 초기 데이터 설정
    for (int row = 0; row < 16; ++row) {
        for (int col = 0; col < 10; ++col) {
            QTableWidgetItem* item = new QTableWidgetItem();
            
            switch (col) {
                case 0: // Status
                    item->setText("N/-/-/0");
                    item->setBackground(QColor(255, 200, 200)); // 연한 빨간색
                    break;
                case 1: // Error
                    item->setText("0/0");
                    break;
                case 2: // Temp
                    item->setText("0/0");
                    break;
                case 3: // Angle Ref
                    item->setText("0.000");
                    break;
                case 4: // Angle Enc
                    item->setText("0.000");
                    break;
                case 5: // Torque Ref
                    item->setText("0.000");
                    break;
                case 6: // Current Sen
                    item->setText("0.000");
                    break;
                case 7: // KP
                    item->setText("0.000");
                    break;
                case 8: // KD
                    item->setText("nan");
                    break;
                case 9: // Owner
                    item->setText("-1");
                    break;
            }
            
            item->setTextAlignment(Qt::AlignCenter);
            item->setFlags(item->flags() & ~Qt::ItemIsEditable); // 읽기 전용
            joint_table->setItem(row, col, item);
        }
    }
    
    // 스타일 설정
    joint_table->setStyleSheet(
        "QTableWidget {"
        "    gridline-color: #d0d0d0;"
        "    background-color: white;"
        "    alternate-background-color: #f5f5f5;"
        "    selection-background-color: #3daee9;"
        "    font-family: 'Courier New', monospace;"
        "    font-size: 10px;"
        "}"
        "QTableWidget::item {"
        "    padding: 2px;"
        "    border: none;"
        "}"
        "QHeaderView::section {"
        "    background-color: #e0e0e0;"
        "    padding: 4px;"
        "    border: 1px solid #d0d0d0;"
        "    font-weight: bold;"
        "}"
    );
    
    // Joint Table을 GroupBox으로 묶기
    QGroupBox* joint_group = new QGroupBox("Joint States");
    joint_group->setObjectName("Joint States");
    QVBoxLayout* joint_group_layout = new QVBoxLayout(joint_group);
    joint_group_layout->addWidget(joint_table);
    advanced_layout->addWidget(joint_group);
    
    // Position Table 생성
    QTableWidget* position_table = new QTableWidget(1, 3);
    position_table->setHorizontalHeaderLabels({
        "X", "Y", "Z"
    });
    
    // 행 이름 설정
    QStringList position_names = {"Position"};
    position_table->setVerticalHeaderLabels(position_names);
    
    // Orientation Table 생성
    QTableWidget* orientation_table = new QTableWidget(1, 4);
    orientation_table->setHorizontalHeaderLabels({
        "X", "Y", "Z", "W"
    });
    
    // 행 이름 설정
    QStringList orientation_names = {"Orientation"};
    orientation_table->setVerticalHeaderLabels(orientation_names);
    
    // Position 테이블 설정
    position_table->setAlternatingRowColors(true);
    position_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    position_table->setSelectionMode(QAbstractItemView::SingleSelection);
    position_table->setSortingEnabled(false);
    
    // Position 헤더 설정
    position_table->horizontalHeader()->setStretchLastSection(false);
    position_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    position_table->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    
    // Position 행 높이 자동 조정
    position_table->resizeRowsToContents();
    
    // Position 테이블 크기 설정
    position_table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    
    // Position 초기 데이터 설정
    for (int col = 0; col < 3; ++col) {
        QTableWidgetItem* item = new QTableWidgetItem();
        item->setText("0.000");
        item->setTextAlignment(Qt::AlignCenter);
        item->setFlags(item->flags() & ~Qt::ItemIsEditable); // 읽기 전용
        position_table->setItem(0, col, item);
    }
    
    // Orientation 테이블 설정
    orientation_table->setAlternatingRowColors(true);
    orientation_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    orientation_table->setSelectionMode(QAbstractItemView::SingleSelection);
    orientation_table->setSortingEnabled(false);
    
    // Orientation 헤더 설정
    orientation_table->horizontalHeader()->setStretchLastSection(false);
    orientation_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    orientation_table->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    
    // Orientation 행 높이 자동 조정
    orientation_table->resizeRowsToContents();
    
    // Orientation 테이블 크기 설정
    orientation_table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    
    // Orientation 초기 데이터 설정
    for (int col = 0; col < 4; ++col) {
        QTableWidgetItem* item = new QTableWidgetItem();
        item->setText("0.000");
        item->setTextAlignment(Qt::AlignCenter);
        item->setFlags(item->flags() & ~Qt::ItemIsEditable); // 읽기 전용
        orientation_table->setItem(0, col, item);
    }
    
    // Position 스타일 설정
    position_table->setStyleSheet(
        "QTableWidget {"
        "    gridline-color: #d0d0d0;"
        "    background-color: white;"
        "    alternate-background-color: #f5f5f5;"
        "    selection-background-color: #3daee9;"
        "    font-family: 'Courier New', monospace;"
        "    font-size: 10px;"
        "}"
        "QTableWidget::item {"
        "    padding: 2px;"
        "    border: none;"
        "}"
        "QHeaderView::section {"
        "    background-color: #e0e0e0;"
        "    padding: 4px;"
        "    border: 1px solid #d0d0d0;"
        "    font-weight: bold;"
        "}"
    );
    
    // Orientation 스타일 설정
    orientation_table->setStyleSheet(
        "QTableWidget {"
        "    gridline-color: #d0d0d0;"
        "    background-color: white;"
        "    alternate-background-color: #f5f5f5;"
        "    selection-background-color: #3daee9;"
        "    font-family: 'Courier New', monospace;"
        "    font-size: 10px;"
        "}"
        "QTableWidget::item {"
        "    padding: 2px;"
        "    border: none;"
        "}"
        "QHeaderView::section {"
        "    background-color: #e0e0e0;"
        "    padding: 4px;"
        "    border: 1px solid #d0d0d0;"
        "    font-weight: bold;"
        "}"
    );
    
    // 테이블 크기 설정
    position_table->setMinimumHeight(60);
    position_table->setMaximumHeight(80);
    orientation_table->setMinimumHeight(60);
    orientation_table->setMaximumHeight(80);
    
    // Robot Pose GroupBox 생성 (Position과 Orientation을 포함)
    QGroupBox* robot_pose_group = new QGroupBox("Robot Pose");
    robot_pose_group->setObjectName("Robot Pose");
    QVBoxLayout* robot_pose_group_layout = new QVBoxLayout(robot_pose_group);
    
    // Position과 Orientation을 수직으로 배치
    robot_pose_group_layout->addWidget(position_table);
    robot_pose_group_layout->addWidget(orientation_table);
    
    advanced_layout->addWidget(robot_pose_group);
    
    // IMU Table 생성
    QTableWidget* imu_table = new QTableWidget(3, 3);
    imu_table->setHorizontalHeaderLabels({
        "angle", "gyro", "acc"
    });
    
    // 행 이름 설정
    QStringList imu_names = {"R(X)", "P(Y)", "Y(Z)"};
    imu_table->setVerticalHeaderLabels(imu_names);
    
    // 테이블 설정
    imu_table->setAlternatingRowColors(true);
    imu_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    imu_table->setSelectionMode(QAbstractItemView::SingleSelection);
    imu_table->setSortingEnabled(false);
    
    // 헤더 설정
    imu_table->horizontalHeader()->setStretchLastSection(false);
    imu_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    imu_table->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    
    // 행 높이 자동 조정
    imu_table->resizeRowsToContents();
    
    // 테이블 크기 설정 - 내용에 맞게 자동 조정
    imu_table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    
    // 초기 데이터 설정
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            QTableWidgetItem* item = new QTableWidgetItem();
            item->setText("0.000");
            item->setTextAlignment(Qt::AlignCenter);
            item->setFlags(item->flags() & ~Qt::ItemIsEditable); // 읽기 전용
            imu_table->setItem(row, col, item);
        }
    }
    
    // 스타일 설정
    imu_table->setStyleSheet(
        "QTableWidget {"
        "    gridline-color: #d0d0d0;"
        "    background-color: white;"
        "    alternate-background-color: #f5f5f5;"
        "    selection-background-color: #3daee9;"
        "    font-family: 'Courier New', monospace;"
        "    font-size: 10px;"
        "}"
        "QTableWidget::item {"
        "    padding: 2px;"
        "    border: none;"
        "}"
        "QHeaderView::section {"
        "    background-color: #e0e0e0;"
        "    padding: 4px;"
        "    border: 1px solid #d0d0d0;"
        "    font-weight: bold;"
        "}"
    );
    
    // 테이블 크기 설정
    imu_table->setMinimumHeight(120);
    imu_table->setMaximumHeight(160);
    
    // IMU Table을 GroupBox으로 묶기
    QGroupBox* imu_group = new QGroupBox("IMU Data");
    imu_group->setObjectName("IMU Data");
    QVBoxLayout* imu_group_layout = new QVBoxLayout(imu_group);
    imu_group_layout->addWidget(imu_table);
    advanced_layout->addWidget(imu_group);
    
    return advanced_tab;
}

void
RbqPanel::onCancelButtonPressed()
{
    timer_.stop();
}

void
RbqPanel::onAccumulatedWp()
{
    std::cout << "Start waypoint" << std::endl;
}

void
RbqPanel::onAccumulatedNTP()
{
    std::cout << "Start navigate through poses" << std::endl;
}

void
RbqPanel::onAccumulating()
{

}

void
RbqPanel::timerEvent(QTimerEvent * event)
{
    Q_UNUSED(event);
}

void
RbqPanel::save(rviz_common::Config config) const
{
    Panel::save(config);
}

void
RbqPanel::load(const rviz_common::Config & config)
{
    Panel::load(config);
}

void
RbqPanel::resetUniqueId()
{
    unique_id = 0;
}

int
RbqPanel::getUniqueId()
{
    int temp_id = unique_id;
    unique_id += 1;
    return temp_id;
}

void
RbqPanel::updateWpNavigationMarkers()
{
    resetUniqueId();
}

template<typename T>
inline std::string RbqPanel::toLabel(T & msg)
{
    return std::string(
        "<tr><td width=150>ETA:</td><td>" +
        toString(rclcpp::Duration(msg.estimated_time_remaining).seconds(), 0) + " s"
                                                                                "</td></tr><tr><td width=150>Distance remaining:</td><td>" +
        toString(msg.distance_remaining, 2) + " m"
                                              "</td></tr><tr><td width=150>Time taken:</td><td>" +
        toString(rclcpp::Duration(msg.navigation_time).seconds(), 0) + " s"
                                                                       "</td></tr><tr><td width=150>Recoveries:</td><td>" +
        std::to_string(msg.number_of_recoveries) +
        "</td></tr>");
}

inline std::string
RbqPanel::toString(double val, int precision)
{
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << val;
    return out.str();
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rbq_rviz_plugins::RbqPanel, rviz_common::Panel)
