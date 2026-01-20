#ifndef THREAD_H
#define THREAD_H

#include <pthread.h>

class Thread {
public:
    static int generate_nrt_thread(pthread_t &thread_nrt, void* (*thread_func)(void *), const char* name, int cpu_no, void *arg);
    static int generate_rt_thread(pthread_t &thread_rt, void* (*thread_func)(void *), const char* name, int cpu_no, int priority, void *arg);
    static int generate_rt_thread_hard(pthread_t &thread_rt, void* (*thread_func)(void *), const char* name, int cpu_no, int priority, void *arg);

    static void timespec_add_us(struct timespec *t, long us){
        t->tv_nsec += us*1000;
        if(t->tv_nsec > 1000000000){
            t->tv_nsec = t->tv_nsec - 1000000000;
            t->tv_sec += 1;
        }
    }
    static int timespec_cmp(struct timespec *a, struct timespec *b){
        if(a->tv_sec > b->tv_sec) return 1;
        else if(a->tv_sec < b->tv_sec) return -1;
        else {
            if(a->tv_nsec > b->tv_nsec) return 1;
            else if(a->tv_nsec == b->tv_nsec) return 0;
            else return -1;
        }
    }
    static int timediff_us(struct timespec *before, struct timespec *after){
        return (after->tv_sec - before->tv_sec)*1e6 + (after->tv_nsec - before->tv_nsec)/1000;
    }
};

#include <chrono>
#ifndef TIMER
#define TIMER
class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }
    static double GetTime() {
        std::chrono::time_point<std::chrono::system_clock> t = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
        return (timestamp*1.0e-9);
    }
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};
#endif // TIMER


#endif // THREAD_H
