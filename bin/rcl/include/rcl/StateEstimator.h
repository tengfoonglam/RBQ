#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <atomic>
#include <mutex>
#include <Eigen/Dense>

struct ROBOT_INFO;
namespace RBQ {
class Estimator;
class ContactEstimator;
};

class StateEstimator {
public:
    static StateEstimator& instance();
    StateEstimator(const StateEstimator&) = delete;
    StateEstimator& operator=(const StateEstimator&) = delete;
    StateEstimator(StateEstimator&&) = delete;
    StateEstimator& operator=(StateEstimator&&) = delete;

    enum Task {
        Task_Idle = 0,
        Task_Estimation_with_contact_input,
        Task_Estimation_with_contact_est,
    };

    void initialize(const int &cpuThread = -1);
    bool isInitialized() { return m_initialized; }

    static void Reset();
    static void Start(const Task &task = Task_Idle);
    static void Stop();
    static Task GetTask() { return m_task; };
    static int Update();

    static void SetCovariance(const std::array<float, 6> &q, const std::array<float, 6> &r);
    static void SetContactThreshold(const float &threshold);

    static int GetState(ROBOT_INFO &outState);

    static void SetContactStatus(const Eigen::Vector4i &contactStatus);
    
private:
    StateEstimator();
    ~StateEstimator();

    static int                      _update();
    static void                     *TaskLoop(void *arg);
    static Task                     m_task;
    static std::atomic<bool>        m_flagTask;

    static float                    m_contactThreshold;
    static Eigen::Vector4i          m_contactStatus;
    static ROBOT_INFO               m_state;

    static bool                     m_initialized;
    static std::mutex               m_mutex;

    static RBQ::Estimator           m_estimator;
    static RBQ::ContactEstimator    m_contactEstimator;
};

#endif // STATE_ESTIMATOR_H
