#ifndef COMMON_STATE_MACHINE_H
#define COMMON_STATE_MACHINE_H

#include <any>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_common {

// 前向声明
template <typename StateType, typename EventType>
class StateMachine;

/**
 * @brief 状态转换结果
 */
enum class TransitionResult {
  kSuccess,            // 转换成功
  kInvalidEvent,       // 无效事件
  kInvalidTransition,  // 无效转换（当前状态不支持该事件）
  kGuardFailed,        // 守卫条件不满足
  kActionFailed,       // 动作执行失败
  kAlreadyInState      // 已经在目标状态
};

/**
 * @brief 状态转换上下文
 */
template <typename StateType, typename EventType>
struct TransitionContext {
  StateType from_state;
  StateType to_state;
  EventType trigger_event;
  std::any event_data;
  uint64_t timestamp_ms;
};

/**
 * @brief 状态配置类
 * 用于配置单个状态的行为
 */
template <typename StateType, typename EventType>
class StateConfig {
 public:
  using GuardFunc = std::function<bool(const TransitionContext<StateType, EventType>&)>;
  using ActionFunc = std::function<void(const TransitionContext<StateType, EventType>&)>;
  using EntryFunc = std::function<void(StateType, const std::any&)>;
  using ExitFunc = std::function<void(StateType, const std::any&)>;

  StateConfig(StateType state, StateMachine<StateType, EventType>* machine);

  /**
   * @brief 设置进入状态时的回调
   */
  StateConfig& OnEntry(EntryFunc func);

  /**
   * @brief 设置退出状态时的回调
   */
  StateConfig& OnExit(ExitFunc func);

  /**
   * @brief 配置状态转换
   * @param event 触发事件
   * @param target_state 目标状态
   */
  StateConfig& AddTransition(EventType event, StateType target_state);

  /**
   * @brief 配置带守卫条件的状态转换
   */
  StateConfig& AddTransition(EventType event, StateType target_state, GuardFunc guard);

  /**
   * @brief 配置带动作的状态转换
   */
  StateConfig& AddTransitionWithAction(EventType event, StateType target_state, ActionFunc action);

  /**
   * @brief 配置带守卫和动作的状态转换
   */
  StateConfig& AddTransitionWithAction(EventType event, StateType target_state, GuardFunc guard, ActionFunc action);

  /**
   * @brief 配置内部转换（不触发进入/退出回调）
   */
  StateConfig& AddInternalTransition(EventType event, ActionFunc action);

  /**
   * @brief 设置为子状态机的入口状态
   */
  StateConfig& AsInitialState();

  /**
   * @brief 设置状态超时
   * @param timeout_ms 超时时间（毫秒）
   * @param timeout_event 超时时触发的事件
   */
  StateConfig& WithTimeout(uint32_t timeout_ms, EventType timeout_event);

 private:
  friend class StateMachine<StateType, EventType>;

  struct Transition {
    StateType target_state;
    GuardFunc guard;
    ActionFunc action;
    bool is_internal = false;
  };

  StateType state_;
  StateMachine<StateType, EventType>* machine_;
  EntryFunc entry_func_;
  ExitFunc exit_func_;
  std::unordered_map<EventType, Transition> transitions_;
  std::optional<uint32_t> timeout_ms_;
  std::optional<EventType> timeout_event_;
};

/**
 * @brief 状态机观察者接口
 */
template <typename StateType, typename EventType>
class IStateMachineObserver {
 public:
  virtual ~IStateMachineObserver() = default;

  virtual void OnStateEntered(StateType /*state*/, const std::any& /*data*/) {}
  virtual void OnStateExited(StateType /*state*/, const std::any& /*data*/) {}
  virtual void OnTransition(const TransitionContext<StateType, EventType>& /*context*/) {}
  virtual void OnTransitionFailed(EventType /*event*/, TransitionResult /*result*/) {}
};

/**
 * @brief 有限状态机类
 *
 * 特性：
 * - 支持守卫条件和转换动作
 * - 支持进入/退出状态回调
 * - 支持状态超时
 * - 支持观察者模式
 * - 线程安全
 */
template <typename StateType, typename EventType>
class StateMachine {
 public:
  using ObserverPtr = std::shared_ptr<IStateMachineObserver<StateType, EventType>>;

  /**
   * @brief 构造函数
   * @param name 状态机名称
   * @param initial_state 初始状态
   */
  explicit StateMachine(const std::string& name, StateType initial_state);
  ~StateMachine();

  // 禁止拷贝
  StateMachine(const StateMachine&) = delete;
  StateMachine& operator=(const StateMachine&) = delete;

  /**
   * @brief 配置状态
   * @param state 要配置的状态
   * @return 状态配置对象
   */
  StateConfig<StateType, EventType>& Configure(StateType state);

  /**
   * @brief 触发事件
   * @param event 事件
   * @param data 事件数据（可选）
   * @return 转换结果
   */
  TransitionResult Fire(EventType event, const std::any& data = {});

  /**
   * @brief 强制设置状态（跳过转换逻辑）
   * 慎用：仅用于错误恢复等特殊场景
   */
  void ForceState(StateType state);

  /**
   * @brief 获取当前状态
   */
  StateType GetCurrentState() const;

  /**
   * @brief 获取上一个状态
   */
  std::optional<StateType> GetPreviousState() const;

  /**
   * @brief 检查是否处于指定状态
   */
  bool IsInState(StateType state) const;

  /**
   * @brief 检查是否可以触发指定事件
   */
  bool CanFire(EventType event) const;

  /**
   * @brief 获取当前状态可触发的事件列表
   */
  std::vector<EventType> GetPermittedEvents() const;

  /**
   * @brief 获取状态机名称
   */
  const std::string& GetName() const;

  /**
   * @brief 添加观察者
   */
  void AddObserver(ObserverPtr observer);

  /**
   * @brief 移除观察者
   */
  void RemoveObserver(ObserverPtr observer);

  /**
   * @brief 重置状态机到初始状态
   */
  void Reset();

 private:
  friend class StateConfig<StateType, EventType>;

  void NotifyStateEntered(StateType state, const std::any& data);
  void NotifyStateExited(StateType state, const std::any& data);
  void NotifyTransition(const TransitionContext<StateType, EventType>& context);
  void NotifyTransitionFailed(EventType event, TransitionResult result);
  uint64_t GetCurrentTimestampMs() const;

  std::string name_;
  StateType initial_state_;
  StateType current_state_;
  std::optional<StateType> previous_state_;

  std::map<StateType, StateConfig<StateType, EventType>> state_configs_;

  mutable std::mutex mutex_;
  std::vector<ObserverPtr> observers_;
};

// ============================================================================
// 模板实现
// ============================================================================

template <typename StateType, typename EventType>
StateConfig<StateType, EventType>::StateConfig(StateType state, StateMachine<StateType, EventType>* machine)
    : state_(state), machine_(machine) {}

template <typename StateType, typename EventType>
StateConfig<StateType, EventType>& StateConfig<StateType, EventType>::OnEntry(EntryFunc func) {
  entry_func_ = std::move(func);
  return *this;
}

template <typename StateType, typename EventType>
StateConfig<StateType, EventType>& StateConfig<StateType, EventType>::OnExit(ExitFunc func) {
  exit_func_ = std::move(func);
  return *this;
}

template <typename StateType, typename EventType>
StateConfig<StateType, EventType>& StateConfig<StateType, EventType>::AddTransition(EventType event,
                                                                                    StateType target_state) {
  transitions_[event] = {target_state, nullptr, nullptr, false};
  return *this;
}

template <typename StateType, typename EventType>
StateConfig<StateType, EventType>& StateConfig<StateType, EventType>::AddTransition(EventType event,
                                                                                    StateType target_state,
                                                                                    GuardFunc guard) {
  transitions_[event] = {target_state, std::move(guard), nullptr, false};
  return *this;
}

template <typename StateType, typename EventType>
StateConfig<StateType, EventType>& StateConfig<StateType, EventType>::AddTransitionWithAction(EventType event,
                                                                                              StateType target_state,
                                                                                              ActionFunc action) {
  transitions_[event] = {target_state, nullptr, std::move(action), false};
  return *this;
}

template <typename StateType, typename EventType>
StateConfig<StateType, EventType>& StateConfig<StateType, EventType>::AddTransitionWithAction(EventType event,
                                                                                              StateType target_state,
                                                                                              GuardFunc guard,
                                                                                              ActionFunc action) {
  transitions_[event] = {target_state, std::move(guard), std::move(action), false};
  return *this;
}

template <typename StateType, typename EventType>
StateConfig<StateType, EventType>& StateConfig<StateType, EventType>::AddInternalTransition(EventType event,
                                                                                            ActionFunc action) {
  transitions_[event] = {state_, nullptr, std::move(action), true};
  return *this;
}

template <typename StateType, typename EventType>
StateConfig<StateType, EventType>& StateConfig<StateType, EventType>::AsInitialState() {
  // 标记为初始状态的逻辑由StateMachine处理
  return *this;
}

template <typename StateType, typename EventType>
StateConfig<StateType, EventType>& StateConfig<StateType, EventType>::WithTimeout(uint32_t timeout_ms,
                                                                                  EventType timeout_event) {
  timeout_ms_ = timeout_ms;
  timeout_event_ = timeout_event;
  return *this;
}

}  // namespace robot_common

#endif  // COMMON_STATE_MACHINE_H
