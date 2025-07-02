
#include "thread_pool.h";

// 示例任务函数
void ThreadPool::exampleTask(int id) {
    std::cout << "Task " << id << " is running." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Task " << id << " finished." << std::endl;
}

void ThreadPool::enqueue(std::function<void()> task) {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        tasks_.push(std::move(task));
    }
    condVar_.notify_one();
}

void ThreadPool::start(size_t numThreads) {
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
}

void ThreadPool::stop() noexcept {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        stop_ = true;
    }
    condVar_.notify_all();
    for (auto& thread : threads_) {
        thread.join();
    }
}


