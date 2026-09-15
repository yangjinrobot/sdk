#ifndef COMMON_TYPE_H_
#define COMMON_TYPE_H_

#include "dridl/gen_types.h"
#include "drdds/core/drdds_core.h"

#define INSTANTIATE_DDS_MSG(MSGTYPE, NAME) \
  template class DrDDSChannel<MSGTYPE>;    \
  template class DrDDSPublisher<MSGTYPE>;  \
  template class DrDDSSubscriber<MSGTYPE>; \
  template class DrDDSMessage<MSGTYPE>;    \
  using Channel##NAME = DrDDSChannel<MSGTYPE>;

#define INSTANTIATE_DDS_SRV(SRVTYPE, NAME)                                                      \
  template class DrDDSServerChannel<SRVTYPE##_RequestPubSubType, SRVTYPE##_ResponsePubSubType>; \
  template class DrDDSClientChannel<SRVTYPE##_RequestPubSubType, SRVTYPE##_ResponsePubSubType>; \
  using ChannelServer##NAME =                                                                   \
      DrDDSServerChannel<SRVTYPE##_RequestPubSubType, SRVTYPE##_ResponsePubSubType>;            \
  using ChannelClient##NAME =                                                                   \
      DrDDSClientChannel<SRVTYPE##_RequestPubSubType, SRVTYPE##_ResponsePubSubType>;

// ===== fibocom msg =====
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::AppCmdPubSubType, AppCmd)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomCameraStatusPubSubType, FibocomCameraStatus)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomOdomPubSubType, FibocomOdom)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomRTKpvtslnPubSubType, FibocomRTKpvtsln)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomRtkenvstatusaPubSubType, FibocomRtkenvstatusa)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomRtkpacklossPubSubType, FibocomRtkpackloss)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::ManageMowPubSubType, ManageMow)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::ManageSetPubSubType, ManageSet)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::TreeNodeStatusPubSubType, TreeNodeStatus)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::BleStatusPubSubType, BleStatus)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomErrorCodePubSubType, FibocomErrorCode)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomOdomChargePubSubType, FibocomOdomCharge)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomRtkPubSubType, FibocomRtk)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomRtkgnggaPubSubType, FibocomRtkgngga)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomRtkstatusaPubSubType, FibocomRtkstatusa)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::ManageOperationPubSubType, ManageOperation)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::RobotConfigPubSubType, RobotConfig)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FiboFusionStatePubSubType, FiboFusionState)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomImuPubSubType, FibocomImu)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomOdomStatusPubSubType, FibocomOdomStatus)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomRtkbestxyzPubSubType, FibocomRtkbestxyz)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomRtkheadingPubSubType, FibocomRtkheading)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::FibocomSensorStatusPubSubType, FibocomSensorStatus)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::ManageRecordPubSubType, ManageRecord)
INSTANTIATE_DDS_MSG(fibocom_msgs::msg::RobotStatusPubSubType, RobotStatus)

// ===== fibocom srv =====
INSTANTIATE_DDS_SRV(fibocom_msgs::srv::FiboFusionInterface, FiboFusionInterface)
INSTANTIATE_DDS_SRV(fibocom_msgs::srv::FiboFusionSetExtrinsic, FiboFusionSetExtrinsic)
INSTANTIATE_DDS_SRV(fibocom_msgs::srv::FiboSaveSpiritConfig, FiboSaveSpiritConfig)

// ===== node msg =====
INSTANTIATE_DDS_MSG(drdds::msg::SetNodePubSubType, SetNode)
INSTANTIATE_DDS_MSG(drdds::msg::NodeStatusPubSubType, NodeStatus)

// ===== Drivers msg =====
INSTANTIATE_DDS_MSG(sensor_msgs::msg::PointCloud2PubSubType, PointCloud2)
using ChannelLidarPointCloud = DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>;
using ChannelGlobalMap = DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>;
INSTANTIATE_DDS_MSG(sensor_msgs::msg::ImuPubSubType, Imu)
INSTANTIATE_DDS_MSG(drdds::msg::NavSatPubSubType, NavSat)
INSTANTIATE_DDS_MSG(sensor_msgs::msg::ImagePubSubType, Image)
INSTANTIATE_DDS_MSG(sensor_msgs::msg::CompressedImagePubSubType, CompressedImage)
INSTANTIATE_DDS_MSG(sensor_msgs::msg::LaserScanPubSubType, LaserScan)
INSTANTIATE_DDS_MSG(sensor_msgs::msg::JointStatePubSubType, JointState)

// ===== Motion msg =====
INSTANTIATE_DDS_MSG(drdds::msg::BatteryDataPubSubType, BatteryData)
INSTANTIATE_DDS_MSG(drdds::msg::CpuDataPubSubType, CpuData)
INSTANTIATE_DDS_MSG(drdds::msg::GaitPubSubType, Gait)
INSTANTIATE_DDS_MSG(drdds::msg::ImuDataPubSubType, ImuData)
INSTANTIATE_DDS_MSG(drdds::msg::JointsDataPubSubType, JointsData)
INSTANTIATE_DDS_MSG(drdds::msg::JointsDataCmdPubSubType, JointsDataCmd)
INSTANTIATE_DDS_MSG(drdds::msg::MotionStatePubSubType, MotionState)
INSTANTIATE_DDS_MSG(drdds::msg::MotionStatusPubSubType, MotionStatus)
INSTANTIATE_DDS_MSG(drdds::msg::MotionInfoPubSubType, MotionInfo)
INSTANTIATE_DDS_MSG(drdds::msg::SteerPubSubType, Steer)
INSTANTIATE_DDS_MSG(drdds::msg::FaultStatusPubSubType, FaultStatus)
INSTANTIATE_DDS_MSG(drdds::msg::JointsPubSubType, Joints)
INSTANTIATE_DDS_MSG(drdds::msg::JointsCmdPubSubType, JointsCmd)

// ===== Location msg =====
INSTANTIATE_DDS_MSG(nav_msgs::msg::OdometryPubSubType, Odometry)
INSTANTIATE_DDS_MSG(geometry_msgs::msg::PoseWithCovarianceStampedPubSubType, InitialPose)
INSTANTIATE_DDS_MSG(drdds::msg::LocationStatusPubSubType, LocationStatus)
INSTANTIATE_DDS_MSG(nav_msgs::msg::OccupancyGridPubSubType, Map)
INSTANTIATE_DDS_MSG(drdds::msg::GridsIDPubSubType, GridsID)

// ===== Plan msg =====
INSTANTIATE_DDS_MSG(geometry_msgs::msg::PoseStampedPubSubType, Goal)
using ChannelPoseStamped = DrDDSChannel<geometry_msgs::msg::PoseStampedPubSubType>;
INSTANTIATE_DDS_MSG(nav_msgs::msg::PathPubSubType, Path)
INSTANTIATE_DDS_MSG(drdds::msg::NavCmdPubSubType, NavCmd)
INSTANTIATE_DDS_MSG(nav_msgs::msg::GridCellsPubSubType, GridCells)
INSTANTIATE_DDS_MSG(drdds::msg::PlannerStatusPubSubType, PlannerStatus)
INSTANTIATE_DDS_MSG(geometry_msgs::msg::TwistPubSubType, Twist)
INSTANTIATE_DDS_MSG(geometry_msgs::msg::PosePubSubType, Pose)

// ===== Navigation msg =====
INSTANTIATE_DDS_MSG(drdds::msg::NavParamPubSubType, NavParam)

// ===== visualization_msgs =====
INSTANTIATE_DDS_MSG(visualization_msgs::msg::ImageMarkerPubSubType, ImageMarker)
INSTANTIATE_DDS_MSG(visualization_msgs::msg::InteractiveMarkerPubSubType, InteractiveMarker)
INSTANTIATE_DDS_MSG(visualization_msgs::msg::InteractiveMarkerControlPubSubType,
                    InteractiveMarkerControl)
INSTANTIATE_DDS_MSG(visualization_msgs::msg::InteractiveMarkerFeedbackPubSubType,
                    InteractiveMarkerFeedback)
INSTANTIATE_DDS_MSG(visualization_msgs::msg::InteractiveMarkerInitPubSubType, InteractiveMarkerInit)
INSTANTIATE_DDS_MSG(visualization_msgs::msg::InteractiveMarkerPosePubSubType, InteractiveMarkerPose)
INSTANTIATE_DDS_MSG(visualization_msgs::msg::InteractiveMarkerUpdatePubSubType,
                    InteractiveMarkerUpdate)
INSTANTIATE_DDS_MSG(visualization_msgs::msg::MarkerPubSubType, Marker)
INSTANTIATE_DDS_MSG(visualization_msgs::msg::MarkerArrayPubSubType, MarkerArray)
INSTANTIATE_DDS_MSG(visualization_msgs::msg::MenuEntryPubSubType, MenuEntry)
INSTANTIATE_DDS_MSG(visualization_msgs::msg::MeshFilePubSubType, MeshFile)
INSTANTIATE_DDS_MSG(visualization_msgs::msg::UVCoordinatePubSubType, UVCoordinate)

// Status
INSTANTIATE_DDS_MSG(drdds::msg::AiryLidarStatusPubSubType, AiryLidarStatus)
INSTANTIATE_DDS_MSG(drdds::msg::LedStatusLightPubSubType, LedStatusLight)
INSTANTIATE_DDS_MSG(drdds::msg::ExceptionPubSubType, Exception)
INSTANTIATE_DDS_MSG(drdds::msg::CpuCoreSuppressCmdPubSubType, CpuCoreSuppressCmd)
INSTANTIATE_DDS_MSG(drdds::msg::CPUStatusPubSubType, CPUStatus)
INSTANTIATE_DDS_MSG(drdds::msg::FileSystemStatusPubSubType, FileSystemStatus)
INSTANTIATE_DDS_MSG(drdds::msg::FaultEventPubSubType, FaultEvent)
INSTANTIATE_DDS_MSG(drdds::msg::FaultEventArrayPubSubType, FaultEventArray)
INSTANTIATE_DDS_MSG(drdds::msg::HeartBeatPubSubType, HeartBeat)
INSTANTIATE_DDS_MSG(drdds::msg::MemoryStatusPubSubType, MemoryStatus)
INSTANTIATE_DDS_MSG(drdds::msg::NetworkStatusPubSubType, NetworkStatus)
INSTANTIATE_DDS_MSG(drdds::msg::ProcessStatusPubSubType, ProcessStatus)

// tf2_msgs
INSTANTIATE_DDS_MSG(tf2_msgs::msg::TFMessagePubSubType, TFMessage)
INSTANTIATE_DDS_MSG(geometry_msgs::msg::TransformStampedPubSubType, TransformStamped)

INSTANTIATE_DDS_MSG(geometry_msgs::msg::PoseArrayPubSubType, PoseArray)

// vision_msgs
INSTANTIATE_DDS_MSG(vision_msgs::msg::Detection3DArrayPubSubType, Detection3DArray)

// std
INSTANTIATE_DDS_MSG(drdds::msg::StdMsgInt32PubSubType, StdMsgInt32)
INSTANTIATE_DDS_MSG(drdds::msg::StdStatusPubSubType, StdStatus)
INSTANTIATE_DDS_MSG(drdds::msg::StdMsgFloat32PubSubType, StdMsgFloat32)
INSTANTIATE_DDS_MSG(std_msgs::msg::BoolPubSubType, Bool)
INSTANTIATE_DDS_MSG(std_msgs::msg::BytePubSubType, Byte)
INSTANTIATE_DDS_MSG(std_msgs::msg::CharPubSubType, Char)
INSTANTIATE_DDS_MSG(std_msgs::msg::EmptyPubSubType, Empty)
INSTANTIATE_DDS_MSG(std_msgs::msg::StringPubSubType, String)
INSTANTIATE_DDS_MSG(std_msgs::msg::Int8PubSubType, Int8)
INSTANTIATE_DDS_MSG(std_msgs::msg::UInt8PubSubType, UInt8)
INSTANTIATE_DDS_MSG(std_msgs::msg::Int16PubSubType, Int16)
INSTANTIATE_DDS_MSG(std_msgs::msg::UInt16PubSubType, UInt16)
INSTANTIATE_DDS_MSG(std_msgs::msg::Int32PubSubType, Int32)
INSTANTIATE_DDS_MSG(std_msgs::msg::UInt32PubSubType, UInt32)
INSTANTIATE_DDS_MSG(std_msgs::msg::Int64PubSubType, Int64)
INSTANTIATE_DDS_MSG(std_msgs::msg::UInt64PubSubType, UInt64)
INSTANTIATE_DDS_MSG(std_msgs::msg::Float32PubSubType, Float32)
INSTANTIATE_DDS_MSG(std_msgs::msg::Float64PubSubType, Float64)
INSTANTIATE_DDS_MSG(std_msgs::msg::HeaderPubSubType, Header)
INSTANTIATE_DDS_MSG(std_msgs::msg::ColorRGBAPubSubType, ColorRGBA)
INSTANTIATE_DDS_MSG(std_msgs::msg::Int8MultiArrayPubSubType, Int8MultiArray)
INSTANTIATE_DDS_MSG(std_msgs::msg::UInt8MultiArrayPubSubType, UInt8MultiArray)
INSTANTIATE_DDS_MSG(std_msgs::msg::Int16MultiArrayPubSubType, Int16MultiArray)
INSTANTIATE_DDS_MSG(std_msgs::msg::UInt16MultiArrayPubSubType, UInt16MultiArray)
INSTANTIATE_DDS_MSG(std_msgs::msg::Int32MultiArrayPubSubType, Int32MultiArray)
INSTANTIATE_DDS_MSG(std_msgs::msg::UInt32MultiArrayPubSubType, UInt32MultiArray)
INSTANTIATE_DDS_MSG(std_msgs::msg::Int64MultiArrayPubSubType, Int64MultiArray)
INSTANTIATE_DDS_MSG(std_msgs::msg::UInt64MultiArrayPubSubType, UInt64MultiArray)
INSTANTIATE_DDS_MSG(std_msgs::msg::Float32MultiArrayPubSubType, Float32MultiArray)
INSTANTIATE_DDS_MSG(std_msgs::msg::Float64MultiArrayPubSubType, Float64MultiArray)

// ===== piper_msgs msg =====
INSTANTIATE_DDS_MSG(piper_msgs::msg::PiperStatusMsgPubSubType, PiperStatusMsg)
INSTANTIATE_DDS_MSG(piper_msgs::msg::PosCmdPubSubType, PosCmd)
// ===== piper_msgs srv =====
INSTANTIATE_DDS_SRV(piper_msgs::srv::Enable, Enable)

// ===== amap_nav_msgs  msg srv =====
INSTANTIATE_DDS_MSG(amap_nav_msgs::msg::AMapNavStatePubSubType, AMapNavState)
INSTANTIATE_DDS_MSG(amap_nav_msgs::msg::WaypointGCJ02PubSubType, WaypointGCJ02)
INSTANTIATE_DDS_SRV(amap_nav_msgs::srv::StartOfflineNavigation, StartOfflineNavigation)
INSTANTIATE_DDS_SRV(amap_nav_msgs::srv::StartOnlineNavigation, StartOnlineNavigation)
INSTANTIATE_DDS_SRV(amap_nav_msgs::srv::PlanPathOffline, PlanPathOffline)

// ===== drdds srv =====
INSTANTIATE_DDS_SRV(drdds::srv::SetParam, SetParam)
INSTANTIATE_DDS_SRV(drdds::srv::SetOOAParam, SetOOAParam)
INSTANTIATE_DDS_SRV(drdds::srv::StdSrvInt32, StdSrvInt32)
INSTANTIATE_DDS_SRV(drdds::srv::StdSrvString, StdSrvString)
INSTANTIATE_DDS_SRV(drdds::srv::JudgeVel, JudgeVel)
INSTANTIATE_DDS_SRV(drdds::srv::LoadMap, LoadMap)
INSTANTIATE_DDS_SRV(drdds::srv::StdSrvStringToInt32, StdSrvStringToInt32)
INSTANTIATE_DDS_SRV(drdds::srv::DeviceControl, DeviceControl)
INSTANTIATE_DDS_SRV(drdds::srv::PoseStampedToInt32, PoseStampedToInt32)
INSTANTIATE_DDS_SRV(drdds::srv::Int32MultiArraySrv, Int32MultiArraySrv)
INSTANTIATE_DDS_SRV(drdds::srv::Float64MultiArraySrv, Float64MultiArraySrv)
INSTANTIATE_DDS_SRV(drdds::srv::BbsRelocalize, BbsRelocalize)
INSTANTIATE_DDS_SRV(drdds::srv::LocApplyReloc, LocApplyReloc)
INSTANTIATE_DDS_SRV(drdds::srv::BbsRelocalizeCancel, BbsRelocalizeCancel)
INSTANTIATE_DDS_SRV(drdds::srv::SetNavParam, SetNavParam)

#endif
