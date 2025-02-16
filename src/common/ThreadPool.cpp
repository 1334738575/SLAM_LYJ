#include "ThreadPool.h"

NSP_SLAM_LYJ_MATH_BEGIN




Thread::Thread(std::queue<Task> *_tasks, std::condition_variable *_con, std::mutex* _mtx, const int _id)
    :m_tasks(_tasks), m_con(_con), m_id(_id), m_mtx(_mtx), std::thread(&Thread::start, this)
{
#ifdef SYS_DEBUG
    std::cout<<"start thread: " << _id << std::endl;
#endif
}
Thread::~Thread()
{}
void Thread::start()
{
    while (true)
    {
        //take
        std::unique_lock<std::mutex> lck(*m_mtx);  
        m_con->wait(lck, [&]{return !m_tasks->empty();});
        auto func = m_tasks->front();
        m_tasks->pop();
        lck.unlock();
        //run
        func();
        //finish
        std::lock_guard<std::mutex> lck2(m_mtx2);
        if(m_toFinish){
            m_isFinish = true;
            break;
        }
    }
    
}
int Thread::getIdInner(){
    return m_id;
}
void Thread::finish()
{
    {
        std::lock_guard<std::mutex> lck(m_mtx2);
        m_toFinish = true;
    }
    while(true){
        std::lock_guard<std::mutex> lck2(m_mtx2);
        if (m_isFinish)
            break;
        _sleep(10);
    }

}



ThreadPool::ThreadPool(const int _threadNum)
{
    m_maxThreadNum = std::thread::hardware_concurrency();
    if(_threadNum <= 0)
        m_threadNum = m_maxThreadNum-3;
    if(m_threadNum <= 0)
        m_threadNum = 1;
#ifdef SYS_DEBUG
    std::cout<<"create thread pool, size: " << m_threadNum << std::endl;
#endif
    m_thds.resize(m_threadNum, nullptr);
    for(int i=0;i<m_threadNum;++i){
        m_thds[i] = new Thread(&m_tasks, &m_con, &m_mtx, i);
        m_thds[i]->detach();
    }
}
ThreadPool::~ThreadPool()
{
    for(int i=0;i<m_threadNum;++i)
    {
        delete m_thds[i];
#ifdef SYS_DEBUG
        std::cout << "delete thread:" << i << std::endl;
#endif // SYS_DEBUG

    }
}
// void ThreadPool::start()
// {
// }
void ThreadPool::addTask(Task _task)
{
    std::lock_guard<std::mutex> lck(m_mtx);
    m_tasks.push(_task);
    m_con.notify_all();
}
void ThreadPool::finish()
{
    while (true) {
        std::lock_guard<std::mutex> lck(m_mtx);
        if (m_tasks.empty()) {
            break;
        }
        _sleep(10);
    }
    for(int i=0;i<m_threadNum;++i)
    {
        m_thds[i]->finish();
#ifdef SYS_DEBUG
        std::cout<<"finishing thread: " <<i << std::endl;
#endif
    }
}
int ThreadPool::maxThreadNum()
{
    return m_maxThreadNum;
}
int ThreadPool::currentThreadNum()
{
    return m_threadNum;
}

NSP_SLAM_LYJ_MATH_END