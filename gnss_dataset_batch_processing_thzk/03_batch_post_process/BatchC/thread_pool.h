#include <iostream>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>

class ThreadPool {
public:
    ThreadPool(size_t numThreads) {
        start(numThreads);
    }

    ~ThreadPool() {
        stop();
    }
    
    void enqueue(std::function<void()> task);/*{
        {
            std::unique_lock<std::mutex> lock(mutex_);
            tasks_.push(std::move(task));
        }
        condVar_.notify_one();
    }*/
    //执行一次线程任务
    void exampleTask(int id);
 
private:
    std::vector<std::thread> threads_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable condVar_;
    bool stop_ = false;

    void start(size_t numThreads); /*{
        for (auto i = 0u; i < numThreads; ++i) {
            threads_.emplace_back([=] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        condVar_.wait(lock, [=] { return stop_ || !tasks_.empty(); });
                        if (stop_ && tasks_.empty()) {
                            return;
                        }
                        task = std::move(tasks_.front());
                        tasks_.pop();
                    }
                    task();
                }
                });
        }
    }*/

    void stop() noexcept;/*{
        {
            std::unique_lock<std::mutex> lock(mutex_);
            stop_ = true;
        }
        condVar_.notify_all();
        for (auto& thread : threads_) {
            thread.join();
        }
    }*/
};
