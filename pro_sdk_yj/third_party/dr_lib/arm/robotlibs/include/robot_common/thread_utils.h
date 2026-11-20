#ifndef COMMON_THREAD_UTILS_H
#define COMMON_THREAD_UTILS_H

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <sys/types.h>
#include <thread>
#include <vector>

namespace robot_common {

/**
 * @brief 线程工具函数命名空间
 */
namespace thread_utils {

/**
 * @brief 设置当前线程名称
 * @param name 线程名称（最长15字符）
 * @return 是否设置成功
 * @note 设置后可通过 htop, top -H, ps -T 等工具查看
 */
bool SetCurrentThreadName(const std::string &name);

/**
 * @brief 获取当前线程名称
 * @return 线程名称
 */
std::string GetCurrentThreadName();

/**
 * @brief 获取当前线程ID (系统级)
 * @return 系统线程ID (tid)
 */
pid_t GetCurrentThreadId();

} // namespace thread_utils

/**
 * @brief 线程优先级枚举
 */
enum class ThreadPriority {
  kLow = 0,
  kNormal = 50,
  kHigh = 80,
  kRealtime = 99
};

/**
 * @brief 线程配置结构体
 */
struct ThreadConfig {
  std::string name = "unnamed_thread";
  ThreadPriority priority = ThreadPriority::kNormal;
  int cpu_affinity = -1; // -1表示不绑定CPU核心
  size_t stack_size = 0; // 0表示使用默认栈大小
};

/**
 * @brief 可管理的线程封装类
 * 提供线程生命周期管理、优先级设置、CPU亲和性绑定等功能
 */
class ManagedThread {
public:
  using TaskFunc = std::function<void()>;

  ManagedThread();
  explicit ManagedThread(const ThreadConfig &config);
  ~ManagedThread();

  // 禁止拷贝
  ManagedThread(const ManagedThread &) = delete;
  ManagedThread &operator=(const ManagedThread &) = delete;

  // 允许移动
  ManagedThread(ManagedThread &&other) noexcept;
  ManagedThread &operator=(ManagedThread &&other) noexcept;

  /**
   * @brief 启动线程执行指定任务
   * @param task 要执行的任务函数
   * @return 是否成功启动
   */
  bool Start(TaskFunc task);

  /**
   * @brief 启动周期性任务
   * @param task 要执行的任务函数
   * @param period_ms 执行周期（毫秒）
   * @return 是否成功启动
   */
  bool StartPeriodic(TaskFunc task, uint32_t period_ms);

  /**
   * @brief 请求停止线程
   */
  void Stop();

  /**
   * @brief 等待线程结束
   */
  void Join();

  /**
   * @brief 检查线程是否正在运行
   */
  bool IsRunning() const;

  /**
   * @brief 获取线程配置
   */
  const ThreadConfig &GetConfig() const;

  /**
   * @brief 获取线程ID
   */
  std::thread::id GetThreadId() const;

private:
  void ApplyThreadConfig();
  void PeriodicRunner(const TaskFunc& task, uint32_t period_ms);

  ThreadConfig config_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  mutable std::mutex mutex_;
};

/**
 * @brief 线程安全的任务队列
 */
template <typename T> class ThreadSafeQueue {
public:
  ThreadSafeQueue() = default;
  ~ThreadSafeQueue() = default;

  void Push(T item) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(std::move(item));
    condition_.notify_one();
  }

  bool Pop(T &item) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    item = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  bool WaitAndPop(T &item, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!condition_.wait_for(lock, timeout,
                             [this] { return !queue_.empty() || shutdown_; })) {
      return false;
    }
    if (shutdown_ && queue_.empty()) {
      return false;
    }
    item = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  void Shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
    condition_.notify_all();
  }

  bool Empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  size_t Size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::queue<T> empty;
    std::swap(queue_, empty);
  }

private:
  mutable std::mutex mutex_;
  std::queue<T> queue_;
  std::condition_variable condition_;
  bool shutdown_ = false;
};

/**
 * @brief 线程池类
 * 支持动态任务提交、优先级任务、任务取消等功能
 */
class ThreadPool {
public:
  /**
   * @brief 构造函数
   * @param num_threads 线程数量，默认为硬件并发数
   * @param pool_name 线程池名称
   */
  explicit ThreadPool(size_t num_threads = 0,
                      const std::string &pool_name = "default_pool");
  ~ThreadPool();

  // 禁止拷贝和移动
  ThreadPool(const ThreadPool &) = delete;
  ThreadPool &operator=(const ThreadPool &) = delete;

  /**
   * @brief 提交任务到线程池
   * @param func 任务函数
   * @param args 任务参数
   * @return future对象，用于获取任务结果
   */
  template <typename F, typename... Args>
  auto Submit(F &&func, Args &&...args)
      -> std::future<typename std::invoke_result<F, Args...>::type> {
    using ReturnType = typename std::invoke_result<F, Args...>::type;

    auto task = std::make_shared<std::packaged_task<ReturnType()>>(
        std::bind(std::forward<F>(func), std::forward<Args>(args)...));

    std::future<ReturnType> result = task->get_future();

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (shutdown_) {
        throw std::runtime_error("ThreadPool已关闭，无法提交新任务");
      }
      tasks_.Push([task]() { (*task)(); });
    }

    return result;
  }

  /**
   * @brief 提交延迟任务
   * @param delay_ms 延迟时间（毫秒）
   * @param func 任务函数
   * @param args 任务参数
   */
  template <typename F, typename... Args>
  auto SubmitDelayed(uint32_t delay_ms, F &&func, Args &&...args)
      -> std::future<typename std::invoke_result<F, Args...>::type> {
    using ReturnType = typename std::invoke_result<F, Args...>::type;

    auto task = std::make_shared<std::packaged_task<ReturnType()>>(
        std::bind(std::forward<F>(func), std::forward<Args>(args)...));

    std::future<ReturnType> result = task->get_future();

    Submit([task, delay_ms]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
      (*task)();
    });

    return result;
  }

  /**
   * @brief 等待所有任务完成
   */
  void WaitAll();

  /**
   * @brief 关闭线程池
   * @param wait_for_tasks 是否等待剩余任务完成
   */
  void Shutdown(bool wait_for_tasks = true);

  /**
   * @brief 获取线程池大小
   */
  size_t Size() const;

  /**
   * @brief 获取待处理任务数量
   */
  size_t PendingTasks() const;

  /**
   * @brief 获取活跃线程数量
   */
  size_t ActiveThreads() const;

  /**
   * @brief 检查线程池是否正在运行
   */
  bool IsRunning() const;

private:
  void WorkerThread(size_t thread_id);

  std::string pool_name_;
  std::vector<std::thread> workers_;
  ThreadSafeQueue<std::function<void()>> tasks_;

  mutable std::mutex queue_mutex_;
  std::condition_variable condition_;
  std::atomic<bool> shutdown_{false};
  std::atomic<size_t> active_threads_{0};
  std::atomic<size_t> completed_tasks_{0};
};

/**
 * @brief 全局线程池单例
 */
class GlobalThreadPool {
public:
  static ThreadPool &Instance();
  static void Initialize(size_t num_threads = 0);
  static void Shutdown();

private:
  GlobalThreadPool() = default;
  static std::unique_ptr<ThreadPool> instance_;
  static std::once_flag init_flag_;
};

} // namespace robot_common

#endif // COMMON_THREAD_UTILS_H
