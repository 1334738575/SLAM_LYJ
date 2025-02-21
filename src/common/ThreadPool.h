#ifndef SLAM_LYJ_THREADPOOL_H
#define SLAM_LYJ_THREADPOOL_H

#include "base/Base.h"
#include <thread>
#include <functional>
#include <condition_variable>
#include <mutex>

NSP_SLAM_LYJ_MATH_BEGIN



using Task = std::function<void()>;

class Thread: public std::thread
{
private:
    /* data */
    std::queue<Task>* m_tasks = nullptr;
    std::condition_variable* m_con = nullptr;
    std::mutex* m_mtx = nullptr;
    int m_id = -1;
    bool m_toFinish = false;
    bool m_isFinish = false;
    std::mutex m_mtx2;
public:
    Thread(/* args */)=delete;

    Thread(std::queue<Task>* _tasks, std::condition_variable* _con, std::mutex* _mtx, const int _id);
    ~Thread();

    void start();
    int getIdInner();
    void finish();
};




class ThreadPool
{
private:
    /* data */
    std::condition_variable m_con;
    std::mutex m_mtx;
    std::queue<Task> m_tasks;
    int m_threadNum = -1;
    int m_maxThreadNum = -1;
    std::vector<Thread*> m_thds;
public:
    ThreadPool(/* args */) = delete;
    ThreadPool(const int _threadNum);
    ~ThreadPool();

    // void start();
    void addTask(Task _task);
    void finish();
    int maxThreadNum();
    int currentThreadNum();
};




NSP_SLAM_LYJ_MATH_END

#endif //SLAM_LYJ_THREADPOOL_H

