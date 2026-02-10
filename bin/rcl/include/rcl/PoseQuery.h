#pragma once

#include <mutex>
#include <atomic>
#include <thread>
#include <asio.hpp>

#include <Eigen/Dense>

#include "rcl/Thread.h"

struct TIME_SYNC {
    double time_query       = 0;
    double time_answer      = 0;
    uint32_t tick_answer    = 0;
    uint32_t tick_query     = 0;
    TIME_SYNC()
    {
        time_query = Timer::GetTime();
    }
    TIME_SYNC(const TIME_SYNC& p)
    {
        time_query  = p.time_query;
        time_answer = p.time_answer;
        tick_answer = p.tick_answer;
        tick_query  = p.tick_query;
    }
    TIME_SYNC& operator=(const TIME_SYNC& p)
    {
        time_query  = p.time_query;
        time_answer = p.time_answer;
        tick_answer = p.tick_answer;
        tick_query  = p.tick_query;
        return *this;
    }
};

struct RobotPose_t
{
    uint32_t tickWrite  = 0;
    uint32_t tickRead   = 0;
    double  time        = Timer::GetTime();
    Eigen::Matrix4f TF  = Eigen::Matrix4f::Identity();
    Eigen::Vector3f rpy = Eigen::Vector3f::Zero();
    std::array<std::array<Eigen::Vector3f, 3>, 4> legs;

    RobotPose_t() { }
    RobotPose_t(const RobotPose_t& p)
    {
        tickWrite   = p.tickWrite;
        tickRead    = p.tickRead;
        time        = p.time;
        TF          = p.TF;
        rpy         = p.rpy;
        legs        = p.legs;
    }
    RobotPose_t& operator=(const RobotPose_t& p)
    {
        tickWrite   = p.tickWrite;
        tickRead    = p.tickRead;
        time        = p.time;
        TF          = p.TF;
        rpy         = p.rpy;
        legs        = p.legs;
        return *this;
    }
    void transformToLocal() {
        Eigen::Matrix3f R = TF.block<3,3>(0,0);
        Eigen::Vector3f t = TF.block<3,1>(0,3);
        Eigen::Matrix3f Rt = R.transpose();
        Eigen::Vector3f tt = -Rt * t;
        // TF.block<3,3>(0,0) = Rt;
        TF.block<3,1>(0,3) = Eigen::Vector3f::Zero();
        for (auto &leg : legs) {
            for (auto &joint : leg) {
                joint = R * (Rt * joint + tt);
            }
        }
    }
};

enum TF_IDs {
    TF_ID_BODY          = 0,
    TF_ID_FOOT_0        = 50,
    TF_ID_FOOT_1        = 51,
    TF_ID_FOOT_2        = 52,
    TF_ID_FOOT_3        = 53,
    TF_ID_BODY_POS      = 54,
    TF_ID_BODY_RPY      = 55,
    TF_ID_CHARGER_CONN  = 200,
};

struct TF_t
{
    double time             = 0;
    uint32_t id             = 0;
    uint32_t tick_answer    = 0;
    uint32_t tick_query     = 0;
    Eigen::Matrix4f local   = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f global  = Eigen::Matrix4f::Identity();
    int     status          = 0;
    TF_t()
    {
        time = Timer::GetTime();
    }
    TF_t(const TF_t& p)
    {
        time        = p.time;
        tick_answer = p.tick_answer;
        tick_query  = p.tick_query;
        status      = p.status;
        id          = p.id;
        local       = p.local;
        global      = p.global;
        status      = p.status;
    }
    TF_t& operator=(const TF_t& p)
    {
        time        = p.time;
        tick_answer = p.tick_answer;
        tick_query  = p.tick_query;
        status      = p.status;
        id          = p.id;
        local       = p.local;
        global      = p.global;
        status      = p.status;
        return *this;
    }
    void set(const uint32_t &_id, const uint32_t &_tick_answer, const uint32_t &_tick_query, const int &_status,
             const Eigen::Matrix4f &_local  = Eigen::Matrix4f::Identity(),
             const Eigen::Matrix4f &_global = Eigen::Matrix4f::Identity())
    {
        id          = _id;
        tick_answer = _tick_answer;
        tick_query  = _tick_query;
        local       = _local;
        global      = _global;
        status      = _status;
    }
};

struct TFPair_t
{
    std::array<TF_t, 4> tfs;
    ///
    /// \brief mode
    /// 0: global frame accumulation & grid update for each sensors
    /// 1: no accumulation & grid when all sensors updated
    int mode = 0;

    TFPair_t() { }
    TFPair_t(const TFPair_t& p)
    {
        tfs = p.tfs;
        mode = p.mode;
    }
    TFPair_t& operator=(const TFPair_t& p)
    {
        tfs = p.tfs;
        mode = p.mode;
        return *this;
    }
};

struct TFs_t
{
    uint32_t tickWrite          = 0;
    uint32_t tickRead           = 0;

    uint32_t senderIp4Addr      = 0;
    unsigned short senderPort   = 0;
    bool senderTcp              = false;
    bool accepted               = false;

    uint32_t tfSize             = 0;
    const static int maxSize    = 10;
    TF_t tfs[maxSize];

    TFs_t() { }

    TFs_t(const TFs_t& p)
    {
        tickWrite   = p.tickWrite;
        tickRead    = p.tickRead;
        senderIp4Addr = p.senderIp4Addr;
        senderPort  = p.senderPort;
        senderTcp   = p.senderTcp;
        accepted    = p.accepted;
        tfSize      = p.tfSize;
        memcpy((char *)tfs, p.tfs, std::min<size_t>(p.tfSize*sizeof(TF_t), sizeof(tfs)));
    }

    TFs_t& operator=(const TFs_t& p)
    {
        tickWrite   = p.tickWrite;
        tickRead    = p.tickRead;
        senderIp4Addr = p.senderIp4Addr;
        senderPort  = p.senderPort;
        senderTcp   = p.senderTcp;
        accepted    = p.accepted;
        tfSize      = p.tfSize;
        memcpy((char *)tfs, p.tfs, std::min<size_t>(p.tfSize*sizeof(TF_t), sizeof(tfs)));

        return *this;
    }
};

constexpr int NUM_OF_POINTS = 77;

struct PointPositions_t
{
    double time = Timer::GetTime();
    std::array<Eigen::Vector3f, NUM_OF_POINTS> positions {Eigen::Vector3f::Zero(),};
    std::array<char, NUM_OF_POINTS> status {0,};

    PointPositions_t() { }
    PointPositions_t(const PointPositions_t& p) {
        time        = p.time;
        positions   = p.positions;
        status      = p.status;
    }
    PointPositions_t& operator=(const PointPositions_t& p) {
        time        = p.time;
        positions   = p.positions;
        status      = p.status;
        return *this;
    }
    float getAnswerZ(const int &_id) {
        return (-1 < _id && _id < NUM_OF_POINTS) ? positions.at(_id).z() : 0;
    }
    Eigen::Vector3f getAnswer(const int &_id) {
        return (-1 < _id && _id < NUM_OF_POINTS) ? positions.at(_id) : Eigen::Vector3f::Zero();
    }
    void setQuery(const int &_id, const Eigen::Vector3f &_pos) {
        if (-1 < _id && _id < NUM_OF_POINTS) {
            status[_id] = -1;
            positions[_id] = _pos;
        }
    }
    float isValid() {
        float ret = 0;
        if(Timer::GetTime() - time < 0.2) {
            for (auto stat : status) {
                if(stat)
                    ret += ((float)stat)/NUM_OF_POINTS;
            }
        }
        // printf("isValid : %.3f , %.3f \n", Timer::GetTime() - time, ret);
        return ret;
    }
};

struct RobotPoseCombined_t
{
    RobotPose_t pose;
    PointPositions_t query;
    int queryMode = 0;
    ///
    /// \brief mode
    /// 0: global frame accumulation & grid update for each sensors
    /// 1: no accumulation & grid when all sensors updated
    int mode = 0;
    RobotPoseCombined_t() {}
    RobotPoseCombined_t(const RobotPoseCombined_t& p) {
        pose = p.pose;
        query = p.query;
        mode = p.mode;
    }
    RobotPoseCombined_t& operator=(const RobotPoseCombined_t& p) {
        pose = p.pose;
        query = p.query;
        mode = p.mode;
        return *this;
    }
};

class PoseQuery
{
public:
    PoseQuery(bool _sim, const std::string &ip = "192.168.0.10");
    ~PoseQuery();

    ///
    /// \brief POINT_SIZE determine max queriable point size
    ///
    static constexpr int POINT_SIZE = 6;

    ///
    /// \brief  Timer to regulate RobotPose_t sending frequency to Vision Heightmap
    ///         Vision Sensors are updated only 15Hz so it's not efficient to send
    ///         these data more than ~50Hz.
    /// \return true if pose timeout
    ///
    bool robotPoseTimeout();

    ///
    /// \brief send robot pose & kinematics to vision
    /// \param _robot_pose
    ///
    void sendRobotPose(const RobotPose_t &_robot_pose);

    ///
    /// \brief send query point position to vision
    /// \param point position
    ///
    void sendPointPositionQuery(const TF_t &_tfQuery);

    ///
    /// \brief send query point position pair to vision
    /// \param point position pair
    ///
    void sendPointPositionQuery(const TFPair_t &_tfQueryPair);

    ///
    /// \brief get if answer is updated
    /// \param idx leg id.
    /// \return true if updated by vision and not timeout
    ///
    bool answerUpdated(int idx);

    ///
    /// \brief get answer point positions
    /// \param idx: leg id.
    /// \return answer point position
    ///
    Eigen::Vector3f answerPosition(int idx);

    ///
    /// \brief get heading angle
    /// \return heading angle wrt stairs in radian
    ///
    float headingAngle() const { return m_headingAngle; }

    ///
    /// \brief send robot pose & kinematics to vision
    /// \param _robot_pose
    ///
    void sendRobotPoseCombined(const RobotPoseCombined_t &_combined);


private:
    void readSocket();
    void setPointPosition(const TF_t &_tf);

    void startSocketVision(const asio::ip::udp::endpoint &_endpoint);
    void stopSocketVision();

    void watcher();

    int _send(const asio::const_buffers_1 buffers);
    int _connected();
private:

    static constexpr int m_robotPoseTimerFrequency = 100;
    int m_fpsRobotPoseSend      = 0;
    int m_fpsQuery              = 0;
    int m_fpsAnswer[POINT_SIZE] = {0,};

    int m_unsentMsgSize;
    int m_unreceivedMsgSize;

    int m_sentByteSize;
    int m_receivedByteSize;

    std::mutex m_answerMutex[POINT_SIZE];
    Timer m_timerAnswer[POINT_SIZE];
    static constexpr double m_answerExpireTimeout   = 0.01;  // 10ms

    Timer m_timerQuery;
    static constexpr double m_queryRegulateTimeout  = 0.01;  // 10ms

    // Timer to regulate RobotPose_t sending to Vision Heightmap
    Timer m_timerRobotPose;
    Timer m_timerDebug;
    Timer m_timerChargerPoseAnswer;
    Timer m_timerChargerPoseQuery;

    bool m_answerUpdated[POINT_SIZE]                = {false,};
    float m_headingAngle                            = 0;
    Eigen::Vector3f m_answerPosition[POINT_SIZE]     = {Eigen::Vector3f::Zero(),};

    // charger pose ---------------------------------------
public:
    ///
    /// \brief send charger pose query to vision
    ///
    void sendChargerPoseQuery();

    ///
    /// \brief get if charger pose is updated
    /// \return true if charger pose updated by vision and not timeout
    ///
    bool chargerPoseUpdated();

    void setChargerPose(const TF_t &_tf);
    Eigen::Matrix4d chargerPose() const { return m_chargerPose; }

    PointPositions_t pointPositions() const { return m_pointPositions; }

private:
    bool m_chargerPoseUpdated = false;
    std::mutex m_chargerPoseMutex;
    Timer m_chargerPoseTimer;
    static constexpr double m_chargerPoseExpireTimeout   = 0.1;  // 100ms

    Eigen::Matrix4d m_chargerPose = Eigen::Matrix4d::Zero();
    // charger pose ---------------------------------------

    std::unique_ptr<asio::ip::udp::socket> m_socket = nullptr;
    asio::io_context m_ioContext;
    std::thread m_ioThread;
    std::atomic<bool> m_ioFlag = 0;

    std::thread m_watcherThread;
    std::atomic<bool> m_watcherFlag = 0;

    std::atomic<bool> m_socketFailed = 0;

    std::string m_ip = "127.0.0.1";

    PointPositions_t m_pointPositions;
    void setPointPositions(PointPositions_t newPointPositions) { m_pointPositions = newPointPositions; }

};
