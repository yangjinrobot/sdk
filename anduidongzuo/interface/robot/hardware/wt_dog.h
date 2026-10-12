#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <cstdint> // Include for int32_t

// Define a type for the callback function
using TimeoutCallback = std::function<void(int32_t, int32_t, double)>;

/// Watchdog class for monitoring and resetting a timer.
class Watchdog {
public:
  // Define constants for time units
  static constexpr int64_t NANOSECONDS_PER_SECOND = 1000000000;

  /**
   * Constructor to create a watchdog timer with a timeout in nanoseconds.
   * @param timeoutNs Timeout interval in nanoseconds.
   * @param callback Callback function to call on timeout.
   */
  Watchdog(int64_t timeoutNs, TimeoutCallback callback);

  /**
   * Default constructor to create a watchdog timer with no initial timeout or callback.
   * Use SetTimeoutAndCallback to set the timeout and callback later.
   */
  Watchdog();

  /// Destructor to release resources.
  ~Watchdog();

  /**
   * Start the watchdog timer.
   * This method starts the watchdog timer in a separate thread.
   */
  void Start();

  /**
   * Feed the watchdog to reset the timer.
   * This method resets the timer interval to the initial value and returns the time remaining until the next timeout.
   * @return Time remaining until the next timeout in timespec format.
   */
  struct timespec Feed();

  /**
   * Stop the watchdog timer.
   * This method stops the watchdog timer and joins the thread.
   */
  void Stop();

  /**
   * Set the timeout interval and callback function of the watchdog timer.
   * This method sets both the timeout interval and the callback function.
   * @param timeoutNs New timeout interval in nanoseconds.
   * @param callback New callback function.
   */
  void SetTimeoutAndCallback(int64_t timeoutNs, TimeoutCallback callback);

private:
  int timer_fd_;
  int64_t timeout_ns_;
  TimeoutCallback callback_;
  std::thread watchdog_thread_;
  std::atomic<bool> running_;
  std::mutex timer_mutex_;

  int32_t timeouts_since_start_;
  int32_t timeouts_since_feed_;
  struct timespec timeout_time_;
  struct timespec last_feed_time_;

  struct itimerspec timer_spec_;

  /**
   * Main function for the watchdog thread.
   * This function monitors the timer and triggers the callback on timeout.
   */
  void Run();
};

#endif  // WATCHDOG_H_
