#include <iostream>
#include <drdds/core/drdds_core.h>
#include <chrono>
#include <ctime>
#include <thread>
#include <set>
#include <map>
#include <sstream>

class DrDDSCtl
{
  private:
  std::shared_ptr<ChannelLidarPointCloud> LidarSub_;
  std::shared_ptr<ChannelOdometry> mLioSub_;
  std::shared_ptr<ChannelJointsData> mJointsData_;
  std::shared_ptr<ChannelJointsDataCmd> mJointsDataCmd_;
  std::shared_ptr<ChannelImuData> mImuData_;
  std::shared_ptr<ChannelImu> mImu_;
  std::shared_ptr<ChannelMotionStatus> mChannelMotionStatus_;
  std::shared_ptr<ChannelMotionInfo> mChannelMotionInfo_;
  std::shared_ptr<ChannelMotionState> mChannelMotionState_;
  std::shared_ptr<ChannelFaultStatus> mChannelFaultStatus_;
  std::shared_ptr<ChannelBatteryData> mChannelBatteryData_;
  std::shared_ptr<ChannelCpuData> mCpu_;
  std::shared_ptr<ChannelSteer> mChannelSteer_;
  std::shared_ptr<ChannelGait> mChannelGait_;
  std::shared_ptr<ChannelNavCmd> mNavCmd_;
  std::shared_ptr<ChannelPlannerStatus> mPlannerStatus_;
  std::shared_ptr<ChannelStdMsgInt32> mStdInt32_;

  static std::queue<std::chrono::system_clock::time_point> TimeQue_;
  static double freq_;

  static void handleJointsData(const drdds::msg::JointsData *msg);
  static void handleOdomtry(const nav_msgs::msg::Odometry *msg);
  static void handleJointsDataCmd(const drdds::msg::JointsDataCmd *msg);
  static void handleImuData(const drdds::msg::ImuData *msg);
  static void handleImu(const sensor_msgs::msg::Imu *msg);
  static void handleLidarData(const sensor_msgs::msg::PointCloud2 *msg);
  static void handleMotionStatus(const drdds::msg::MotionStatus *msg);
  static void handleMotionInfo(const drdds::msg::MotionInfo *msg);
  static void handleMotionState(const drdds::msg::MotionState *msg);
  static void handleFaultStatus(const drdds::msg::FaultStatus *msg);
  static void handleExeHealth(const drdds::msg::BatteryData *msg);
  static void handleExeHealthCpu(const drdds::msg::CpuData *msg);
  static void handleSteer(const drdds::msg::Steer *msg);
  static void handleGait(const drdds::msg::Gait *msg);
  static void handleMetaType(const drdds::msg::MetaType &msg);
  static void handleStdMsgsHead(const std_msgs::msg::Header &msg);
  static void handleNavCmd(const drdds::msg::NavCmd *msg);
  static void handlePlannerStatus(const drdds::msg::PlannerStatus *msg);
  static void handleStdMsgInt32(const drdds::msg::StdMsgInt32 *msg);
  static void GetFreq();

  public:
  DrDDSCtl(/* args */);
  ~DrDDSCtl();

  void echo(std::string topic);
  void list();
  static std::string cmd_;
  static int wsize_;
  static double curtime;

  class CustomParticipantListener : public eprosima::fastdds::dds::DomainParticipantListener
  {
public:
    struct NodeGraph
    {
      std::set<std::string> publishers;
      std::set<std::string> subscribers;
    };

    // 使用参与者 GUID 前缀作为唯一节点 ID
    std::map<std::string, NodeGraph> graph;

    void on_publisher_discovery(eprosima::fastdds::dds::DomainParticipant * /*participant*/,
                                eprosima::fastrtps::rtps::WriterDiscoveryInfo &&info) override
    {
      if (info.status != eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER) return;

      std::lock_guard<std::mutex> lock(mutex_);
      std::string node_id = guid_prefix_to_string(info.info.guid().guidPrefix); // 只取前缀
      std::string topic = static_cast<std::string>(info.info.topicName());

      if (!is_user_topic(topic)) return;

      graph[node_id].publishers.insert(topic);
      std::cout << "Publisher: " << node_id << " -> " << topic << std::endl;
    }

    void on_subscriber_discovery(eprosima::fastdds::dds::DomainParticipant * /*participant*/,
                                 eprosima::fastrtps::rtps::ReaderDiscoveryInfo &&info) override
    {
      if (info.status != eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER) return;

      std::lock_guard<std::mutex> lock(mutex_);
      std::string node_id = guid_prefix_to_string(info.info.guid().guidPrefix); // 只取前缀
      std::string topic = static_cast<std::string>(info.info.topicName());

      if (!is_user_topic(topic)) return;

      graph[node_id].subscribers.insert(topic);
      std::cout << "Subscriber: " << node_id << " -> " << topic << std::endl;
    }

    // 输出 DOT 文件
    std::string dump_dot() const
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::ostringstream oss;
      oss << "digraph DDS {\n";
      oss << "  rankdir=LR;\n"; // 左到右布局

      std::set<std::string> all_topics;
      for (const auto &[node_id, ng] : graph) {
        all_topics.insert(ng.publishers.begin(), ng.publishers.end());
        all_topics.insert(ng.subscribers.begin(), ng.subscribers.end());
      }

      // 输出 topic 节点（椭圆）
      for (const auto &topic : all_topics) {
        if (!is_user_topic(topic)) continue;
        oss << "  \"" << topic << "\" [shape=ellipse, style=filled, fillcolor=lightgray];\n";
      }

      // 输出 node 节点（方框）并连接 topic
      for (const auto &[node_id, ng] : graph) {
        oss << "  \"" << node_id << "\" [shape=box, style=filled, color=lightblue];\n";

        // publisher -> topic
        for (const auto &topic : ng.publishers) {
          if (!is_user_topic(topic)) continue;
          oss << "  \"" << node_id << "\" -> \"" << topic << "\" [color=blue];\n";
        }

        // topic -> subscriber
        for (const auto &topic : ng.subscribers) {
          if (!is_user_topic(topic)) continue;
          oss << "  \"" << topic << "\" -> \"" << node_id << "\" [color=red];\n";
        }
      }

      oss << "}\n";
      return oss.str();
    }

private:
    mutable std::mutex mutex_;

    // 将 GUID 前缀转换为字符串
    static std::string guid_prefix_to_string(const eprosima::fastrtps::rtps::GuidPrefix_t &prefix)
    {
      std::ostringstream oss;
      oss << std::hex;
      for (int i = 0; i < 12; ++i) {
        oss << (int)prefix.value[i];
      }
      return oss.str();
    }

    // 系统话题和 request/reply 类型过滤
    static bool is_user_topic(const std::string &topic)
    {
      // 去掉系统话题
      if (topic == "ros_discovery_info" || topic == "rt/parameter_events") return false;

      // 去掉 rq/ rr/ 类型话题
      if (topic.find("rq/") == 0 || topic.find("rr/") == 0) return false;

      return true; // 保留普通话题
    }
  };
};
