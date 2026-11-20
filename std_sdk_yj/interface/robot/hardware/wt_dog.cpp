#include "wt_dog.h"
#include <iostream>
#include <sys/timerfd.h>
#include <unistd.h>
#include <sys/epoll.h>
#include <cstring>

Watchdog::Watchdog(int64_t timeoutNs, TimeoutCallback callback)
    : timeout_ns_(timeoutNs), callback_(callback), running_(false),
      timeouts_since_start_(0), timeouts_since_feed_(0) {
  // 创建 timerfd
  timer_fd_ = timerfd_create(CLOCK_MONOTONIC, 0);
  if (timer_fd_ == -1) {
    perror("timerfd_create");
    exit(EXIT_FAILURE);
  }

  // Set the initial timeout value
  timer_spec_.it_value.tv_sec = timeout_ns_ / NANOSECONDS_PER_SECOND; // Convert ns to s
  timer_spec_.it_value.tv_nsec = timeout_ns_ % NANOSECONDS_PER_SECOND;
  timer_spec_.it_interval = timer_spec_.it_value;

  if (timerfd_settime(timer_fd_, 0, &timer_spec_, nullptr) == -1) {
    perror("timerfd_settime");
    exit(EXIT_FAILURE);
  }
}

Watchdog::Watchdog()
    : timer_fd_(-1), timeout_ns_(0), callback_(nullptr), running_(false),
      timeouts_since_start_(0), timeouts_since_feed_(0) {
  // 默认构造函数，不设置初始参数
  timer_fd_ = timerfd_create(CLOCK_MONOTONIC, 0);
  if (timer_fd_ == -1) {
    perror("timerfd_create");
    exit(EXIT_FAILURE);
  }
}

Watchdog::~Watchdog() {
  if (timer_fd_ != -1) {
    close(timer_fd_);
  }
}

void Watchdog::Start() {
  if (running_) {
    return;
  }

  running_ = true;
  watchdog_thread_ = std::thread(&Watchdog::Run, this);
}

struct timespec Watchdog::Feed() {
  std::lock_guard<std::mutex> lock(timer_mutex_);

  // Calculate remaining time until the next timeout
  struct itimerspec current_timer_spec;
  if (timerfd_gettime(timer_fd_, &current_timer_spec) == -1) {
    perror("timerfd_gettime");
    exit(EXIT_FAILURE);
  }

  // Reset the timer to the initial timeout value
  if (timerfd_settime(timer_fd_, 0, &timer_spec_, nullptr) == -1) {
    perror("timerfd_settime");
    exit(EXIT_FAILURE);
  }
  clock_gettime(CLOCK_MONOTONIC, &last_feed_time_);
  // timeouts_since_start_++;
  timeouts_since_feed_ = 0;

  return current_timer_spec.it_value;
}

void Watchdog::Stop() {
  if (running_) {
    running_ = false;
    if (watchdog_thread_.joinable()) {
      watchdog_thread_.detach();
    }
  }
}

void Watchdog::SetTimeoutAndCallback(int64_t timeoutNs, TimeoutCallback callback) {
  std::cout << timer_fd_ << std::endl;
  std::lock_guard<std::mutex> lock(timer_mutex_);
  timeout_ns_ = timeoutNs;
  callback_ = callback;

  // Reset the timer to the new timeout value
  // timer_spec_.it_value.tv_sec = timeout_ns_ / NANOSECONDS_PER_SECOND; // Convert ns to s
  // timer_spec_.it_value.tv_nsec = timeout_ns_ % NANOSECONDS_PER_SECOND;
  // timer_spec_.it_interval = timer_spec_.it_value;
  timer_spec_.it_value.tv_sec = timeout_ns_ / NANOSECONDS_PER_SECOND; // Convert ns to s
  timer_spec_.it_value.tv_nsec = timeout_ns_ % NANOSECONDS_PER_SECOND;
  timer_spec_.it_interval = timer_spec_.it_value;

  if (timerfd_settime(timer_fd_, 0, &timer_spec_, nullptr) == -1) {
    perror("timerfd_settime");
    exit(EXIT_FAILURE);
  }
  // Feed();
}

void Watchdog::Run() {
  if (timer_fd_ == -1) {
    std::cerr << "Watchdog is not properly initialized. Please set timeout and callback." << std::endl;
    return;
  }

  // Create epoll instance
  int epoll_fd = epoll_create1(0);
  if (epoll_fd == -1) {
    perror("epoll_create1");
    exit(EXIT_FAILURE);
  }

  // Add timer_fd_ to epoll
  struct epoll_event event;
  event.events = EPOLLIN;
  event.data.fd = timer_fd_;
  if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_fd_, &event) == -1) {
    perror("epoll_ctl");
    exit(EXIT_FAILURE);
  }

  while (running_) {
    // Wait for events
    struct epoll_event events[1];
    int num_events = epoll_wait(epoll_fd, events, 1, -1);
    if (num_events == -1) {
      perror("epoll_wait");
      exit(EXIT_FAILURE);
    }

    if (events[0].data.fd == timer_fd_) {
      // Timer timeout event occurred
      uint64_t expirations;
      ssize_t s = read(timer_fd_, &expirations, sizeof(expirations));
      if (s != sizeof(expirations)) {
        perror("read");
        exit(EXIT_FAILURE);
      }

      std::lock_guard<std::mutex> lock(timer_mutex_);
      timeouts_since_start_++;
      timeouts_since_feed_++;

      if (callback_ && clock_gettime(CLOCK_MONOTONIC, &timeout_time_) == -1) {
        perror("clock_gettime");
        exit(EXIT_FAILURE);
      }
      double elapsed_time = (timeout_time_.tv_sec - last_feed_time_.tv_sec) +
                            (timeout_time_.tv_nsec - last_feed_time_.tv_nsec) / 1e9;
      if (callback_) {
        callback_(timeouts_since_start_, timeouts_since_feed_, elapsed_time);
      }
    }
  }

  close(epoll_fd);
}
