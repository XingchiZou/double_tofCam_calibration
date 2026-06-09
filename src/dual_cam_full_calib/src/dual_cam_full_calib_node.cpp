/**
 * dual_cam_full_calib_node.cpp
 *
 * 双相机全标定节点：利用"纯原地旋转"与"纯笔直前行"两种刚体运动，
 * 通过 ICP 点云配准 + 多阶段数学求解，在线计算：
 *   - 姿态纠正矩阵 R_align (消除安装误差)
 *   - 各相机水平力臂 r1, r2
 *   - 双目外参 X = T_{C1←C2}
 *   - 安装偏航角 ψ
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <vector>
#include <cmath>
#include <sstream>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class DualCamFullCalibNode
{
public:
  DualCamFullCalibNode()
    : nh_()
    , pnh_("~")
  {
  }

  void init()
  {
    loadParameters();

    sub_cam1_ = nh_.subscribe(cam1_topic_, 1, &DualCamFullCalibNode::cam1Callback, this);
    sub_cam2_ = nh_.subscribe(cam2_topic_, 1, &DualCamFullCalibNode::cam2Callback, this);

    srv_set_ref_      = pnh_.advertiseService("set_reference",      &DualCamFullCalibNode::setReference, this);
    srv_add_pivot_    = pnh_.advertiseService("add_pivot_sample",   &DualCamFullCalibNode::addPivotSample, this);
    srv_add_straight_ = pnh_.advertiseService("add_straight_sample",&DualCamFullCalibNode::addStraightSample, this);
    srv_calibrate_    = pnh_.advertiseService("calibrate",          &DualCamFullCalibNode::calibrate, this);

    ROS_INFO("dual_cam_full_calib initialized, waiting for point clouds and service calls...");
  }

private:
  // ---------------------------------------------------------------------------
  //  参数加载
  // ---------------------------------------------------------------------------
  void loadParameters()
  {
    pnh_.param<std::string>("cam1_topic", cam1_topic_, "/cam1/points");
    pnh_.param<std::string>("cam2_topic", cam2_topic_, "/cam2/points");
    pnh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.05);
    pnh_.param<int>("icp_max_iterations", icp_max_iterations_, 50);
    pnh_.param<double>("icp_transformation_epsilon", icp_transformation_epsilon_, 1e-8);
    pnh_.param<double>("icp_max_correspondence_distance", icp_max_correspondence_distance_, 0.5);
    pnh_.param<int>("min_pivot_samples", min_pivot_samples_, 2);
    pnh_.param<int>("min_straight_samples", min_straight_samples_, 1);

    ROS_INFO("Parameters loaded:");
    ROS_INFO("  cam1_topic: %s", cam1_topic_.c_str());
    ROS_INFO("  cam2_topic: %s", cam2_topic_.c_str());
    ROS_INFO("  voxel_leaf_size: %.4f", voxel_leaf_size_);
    ROS_INFO("  icp_max_iterations: %d", icp_max_iterations_);
    ROS_INFO("  icp_transformation_epsilon: %.2e", icp_transformation_epsilon_);
    ROS_INFO("  icp_max_correspondence_distance: %.4f", icp_max_correspondence_distance_);
    ROS_INFO("  min_pivot_samples: %d", min_pivot_samples_);
    ROS_INFO("  min_straight_samples: %d", min_straight_samples_);
  }

  // ---------------------------------------------------------------------------
  //  点云回调
  // ---------------------------------------------------------------------------
  void cam1Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_1_ = msg;
  }

  void cam2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_2_ = msg;
  }

  // ---------------------------------------------------------------------------
  //  PCL 工具
  // ---------------------------------------------------------------------------
  CloudT::Ptr toPcl(const sensor_msgs::PointCloud2::ConstPtr& msg) const
  {
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    return cloud;
  }

  CloudT::Ptr voxelDownsample(const CloudT::ConstPtr& input) const
  {
    CloudT::Ptr filtered(new CloudT);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(input);
    const float leaf = static_cast<float>(voxel_leaf_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*filtered);
    return filtered;
  }

  bool runIcp(const CloudT::ConstPtr& source, const CloudT::ConstPtr& target, Eigen::Matrix4d& transform) const
  {
    CloudT::Ptr src = voxelDownsample(source);
    CloudT::Ptr tgt = voxelDownsample(target);

    if (src->empty() || tgt->empty())
    {
      ROS_WARN("ICP: empty cloud after downsampling (src=%zu, tgt=%zu)", src->size(), tgt->size());
      return false;
    }

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setTransformationEpsilon(icp_transformation_epsilon_);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);

    CloudT aligned;
    icp.align(aligned);

    if (!icp.hasConverged())
    {
      ROS_WARN("ICP did not converge, discarding this sample");
      return false;
    }

    transform = icp.getFinalTransformation().cast<double>();
    return true;
  }

  // ---------------------------------------------------------------------------
  //  服务：set_reference
  // ---------------------------------------------------------------------------
  bool setReference(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!latest_cloud_1_ || !latest_cloud_2_)
    {
      res.success = false;
      res.message = "Point clouds from both cameras have not been received";
      return true;
    }

    ref_cloud_1_ = toPcl(latest_cloud_1_);
    ref_cloud_2_ = toPcl(latest_cloud_2_);
    pivot_samples_.clear();
    straight_samples_.clear();
    ROS_INFO("Observation queues cleared");

    res.success = true;
    res.message = "Reference frame updated";
    ROS_INFO("%s", res.message.c_str());
    return true;
  }

  // ---------------------------------------------------------------------------
  //  服务：add_pivot_sample (旋转观测)
  // ---------------------------------------------------------------------------
  bool addPivotSample(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!ref_cloud_1_ || !ref_cloud_2_)
    {
      res.success = false;
      res.message = "Please call set_reference first";
      return true;
    }
    if (!latest_cloud_1_ || !latest_cloud_2_)
    {
      res.success = false;
      res.message = "Latest point clouds have not been received";
      return true;
    }

    CloudT::Ptr cur1 = toPcl(latest_cloud_1_);
    CloudT::Ptr cur2 = toPcl(latest_cloud_2_);

    Eigen::Matrix4d A_k = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d B_k = Eigen::Matrix4d::Identity();

    if (!runIcp(cur1, ref_cloud_1_, A_k) || !runIcp(cur2, ref_cloud_2_, B_k))
    {
      res.success = false;
      res.message = "ICP failed, pivot sample not added";
      return true;
    }

    pivot_samples_.push_back({A_k, B_k});

    std::ostringstream oss;
    oss << "Pivot samples collected: " << pivot_samples_.size();
    res.success = true;
    res.message = oss.str();
    ROS_INFO("%s", res.message.c_str());
    return true;
  }

  // ---------------------------------------------------------------------------
  //  服务：add_straight_sample (直行观测，仅 Cam1)
  // ---------------------------------------------------------------------------
  bool addStraightSample(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!ref_cloud_1_)
    {
      res.success = false;
      res.message = "Please call set_reference first";
      return true;
    }
    if (!latest_cloud_1_)
    {
      res.success = false;
      res.message = "Latest Cam1 point cloud has not been received";
      return true;
    }

    CloudT::Ptr cur1 = toPcl(latest_cloud_1_);
    Eigen::Matrix4d T_straight = Eigen::Matrix4d::Identity();

    if (!runIcp(cur1, ref_cloud_1_, T_straight))
    {
      res.success = false;
      res.message = "ICP failed, straight sample not added";
      return true;
    }

    straight_samples_.push_back(T_straight);

    std::ostringstream oss;
    oss << "Straight samples collected: " << straight_samples_.size();
    res.success = true;
    res.message = oss.str();
    ROS_INFO("%s", res.message.c_str());
    return true;
  }

  // ---------------------------------------------------------------------------
  //  阶段一：计算姿态纠正矩阵 R_align
  // ---------------------------------------------------------------------------
  Eigen::Matrix3d computeAlignRotation(const std::vector<Eigen::Matrix4d>& raw_transforms) const
  {
    const Eigen::Vector3d v_target(-1.0, 0.0, 0.0);

    Eigen::Vector3d axis_sum = Eigen::Vector3d::Zero();
    for (const auto& T : raw_transforms)
    {
      Eigen::AngleAxisd aa(T.block<3, 3>(0, 0));
      axis_sum += aa.axis();
    }

    Eigen::Vector3d u_true = axis_sum.normalized();

    if (u_true.dot(v_target) < 0.0)
      u_true = -u_true;

    return Eigen::Quaterniond::FromTwoVectors(u_true, v_target).toRotationMatrix();
  }

  // ---------------------------------------------------------------------------
  //  阶段二：修正观测轨迹
  //  R_corrected = R_align * R_raw * R_align^T
  //  t_corrected = R_align * t_raw
  // ---------------------------------------------------------------------------
  Eigen::Matrix4d correctTransform(const Eigen::Matrix3d& R_align, const Eigen::Matrix4d& T_raw) const
  {
    const Eigen::Matrix3d R_raw = T_raw.block<3, 3>(0, 0);
    const Eigen::Vector3d t_raw = T_raw.block<3, 1>(0, 3);

    Eigen::Matrix4d T_corr = Eigen::Matrix4d::Identity();
    T_corr.block<3, 3>(0, 0) = R_align * R_raw * R_align.transpose();
    T_corr.block<3, 1>(0, 3) = R_align * t_raw;
    return T_corr;
  }

  // ---------------------------------------------------------------------------
  //  阶段三：水平力臂求解 (降维 2D 最小二乘)
  // ---------------------------------------------------------------------------
  Eigen::Vector2d solveHorizontalLeverArm(const std::vector<Eigen::Matrix4d>& transforms) const
  {
    const int N = static_cast<int>(transforms.size());
    Eigen::MatrixXd M(2 * N, 2);
    Eigen::VectorXd b(2 * N);

    for (int k = 0; k < N; ++k)
    {
      const Eigen::Matrix3d R = transforms[k].block<3, 3>(0, 0);
      const Eigen::Vector3d t = transforms[k].block<3, 1>(0, 3);
      M.block<2, 2>(2 * k, 0) = R.block<2, 2>(0, 0) - Eigen::Matrix2d::Identity();
      b(2 * k)     = t.x();
      b(2 * k + 1) = t.y();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.solve(b);
  }

  // ---------------------------------------------------------------------------
  //  阶段四辅助：四元数左/右乘矩阵
  // ---------------------------------------------------------------------------
  static Eigen::Matrix4d leftQuatMatrix(const Eigen::Quaterniond& q)
  {
    Eigen::Matrix4d L;
    L << q.w(), -q.x(), -q.y(), -q.z(),
         q.x(),  q.w(), -q.z(),  q.y(),
         q.y(),  q.z(),  q.w(), -q.x(),
         q.z(), -q.y(),  q.x(),  q.w();
    return L;
  }

  static Eigen::Matrix4d rightQuatMatrix(const Eigen::Quaterniond& q)
  {
    Eigen::Matrix4d R;
    R << q.w(), -q.x(), -q.y(), -q.z(),
         q.x(),  q.w(),  q.z(), -q.y(),
         q.y(), -q.z(),  q.w(),  q.x(),
         q.z(),  q.y(), -q.x(),  q.w();
    return R;
  }

  // ---------------------------------------------------------------------------
  //  阶段四：外参旋转求解 (AX = XB)
  // ---------------------------------------------------------------------------
  Eigen::Quaterniond solveRotationAXEqualsXB(
    const std::vector<Eigen::Matrix4d>& A_list,
    const std::vector<Eigen::Matrix4d>& B_list) const
  {
    const int N = static_cast<int>(A_list.size());
    Eigen::MatrixXd K(4 * N, 4);
    K.setZero();

    for (int k = 0; k < N; ++k)
    {
      const Eigen::Quaterniond q_A(A_list[k].block<3, 3>(0, 0));
      const Eigen::Quaterniond q_B(B_list[k].block<3, 3>(0, 0));
      K.block<4, 4>(4 * k, 0) = leftQuatMatrix(q_A) - rightQuatMatrix(q_B);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(K, Eigen::ComputeFullV);
    const Eigen::Vector4d q_vec = svd.matrixV().col(3);
    Eigen::Quaterniond q(q_vec(3), q_vec(0), q_vec(1), q_vec(2));
    q.normalize();
    return q;
  }

  // ---------------------------------------------------------------------------
  //  阶段四：外参平移求解
  // ---------------------------------------------------------------------------
  Eigen::Vector3d solveTranslation(
    const std::vector<Eigen::Matrix4d>& A_list,
    const std::vector<Eigen::Matrix4d>& B_list,
    const Eigen::Matrix3d& R_X) const
  {
    const int N = static_cast<int>(A_list.size());
    Eigen::MatrixXd M(3 * N, 3);
    Eigen::VectorXd b(3 * N);

    for (int k = 0; k < N; ++k)
    {
      const Eigen::Matrix3d R_A = A_list[k].block<3, 3>(0, 0);
      const Eigen::Vector3d t_A = A_list[k].block<3, 1>(0, 3);
      const Eigen::Vector3d t_B = B_list[k].block<3, 1>(0, 3);

      M.block<3, 3>(3 * k, 0) = R_A - Eigen::Matrix3d::Identity();
      b.segment<3>(3 * k) = R_X * t_B - t_A;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.solve(b);
  }

  // ---------------------------------------------------------------------------
  //  阶段五：偏航纠正角 ψ
  // ---------------------------------------------------------------------------
  double solveYawAngle(const Eigen::Matrix3d& R_align1,
                       const std::vector<Eigen::Matrix4d>& straight_raw) const
  {
    Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();
    for (const auto& T : straight_raw)
    {
      t_sum += R_align1 * T.block<3, 1>(0, 3);
    }
    Eigen::Vector3d t_avg = t_sum / static_cast<double>(straight_raw.size());
    return std::atan2(t_avg.y(), t_avg.z());
  }

  // ---------------------------------------------------------------------------
  //  服务：calibrate (六阶段标定)
  // ---------------------------------------------------------------------------
  bool calibrate(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (static_cast<int>(pivot_samples_.size()) < min_pivot_samples_)
    {
      std::ostringstream err;
      err << "Not enough pivot samples (have " << pivot_samples_.size()
          << ", need >= " << min_pivot_samples_ << ")";
      res.success = false;
      res.message = err.str();
      return true;
    }
    if (static_cast<int>(straight_samples_.size()) < min_straight_samples_)
    {
      std::ostringstream err;
      err << "Not enough straight samples (have " << straight_samples_.size()
          << ", need >= " << min_straight_samples_ << ")";
      res.success = false;
      res.message = err.str();
      return true;
    }

    // 收集原始观测
    std::vector<Eigen::Matrix4d> A_raw, B_raw;
    for (const auto& s : pivot_samples_)
    {
      A_raw.push_back(s.A);
      B_raw.push_back(s.B);
    }

    // ====== 阶段一：计算 R_align ======
    const Eigen::Matrix3d R_align1 = computeAlignRotation(A_raw);
    const Eigen::Matrix3d R_align2 = computeAlignRotation(B_raw);

    // ====== 阶段二：修正观测轨迹 ======
    std::vector<Eigen::Matrix4d> A_corr, B_corr;
    for (size_t i = 0; i < A_raw.size(); ++i)
    {
      A_corr.push_back(correctTransform(R_align1, A_raw[i]));
      B_corr.push_back(correctTransform(R_align2, B_raw[i]));
    }

    // ====== 阶段三：水平力臂求解 ======
    const Eigen::Vector2d r1 = solveHorizontalLeverArm(A_corr);
    const Eigen::Vector2d r2 = solveHorizontalLeverArm(B_corr);

    // ====== 阶段四：外参求解 AX=XB ======
    const Eigen::Quaterniond q_X = solveRotationAXEqualsXB(A_corr, B_corr);
    const Eigen::Matrix3d R_X = q_X.toRotationMatrix();
    const Eigen::Vector3d t_X = solveTranslation(A_corr, B_corr, R_X);

    Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
    X.block<3, 3>(0, 0) = R_X;
    X.block<3, 1>(0, 3) = t_X;

    // ====== 阶段五：偏航纠正角 ψ ======
    const double psi_rad = solveYawAngle(R_align1, straight_samples_);
    const double psi_deg = psi_rad * 180.0 / M_PI;

    // ====== 阶段六：终端输出 ======
    std::ostringstream oss;
    oss << "\n============ calibration results ============\n"
        << "R_align1 (Cam1 alignment rotation):\n" << R_align1 << "\n\n"
        << "R_align2 (Cam2 alignment rotation):\n" << R_align2 << "\n\n"
        << "Camera 1 lever arm: r1x=" << r1.x() << ", r1y=" << r1.y() << "\n"
        << "Camera 2 lever arm: r2x=" << r2.x() << ", r2y=" << r2.y() << "\n\n"
        << "Extrinsic X (T_C1<-C2):\n" << X << "\n\n"
        << "Yaw correction psi = " << psi_deg << " deg (" << psi_rad << " rad)\n"
        << "=============================================";

    ROS_INFO("%s", oss.str().c_str());

    res.success = true;
    res.message = "Calibration completed, results have been output to the terminal";
    return true;
  }

  // ---------------------------------------------------------------------------
  //  数据结构
  // ---------------------------------------------------------------------------
  struct PivotSample
  {
    Eigen::Matrix4d A;
    Eigen::Matrix4d B;
  };

  // ---------------------------------------------------------------------------
  //  成员变量
  // ---------------------------------------------------------------------------
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_cam1_;
  ros::Subscriber sub_cam2_;
  ros::ServiceServer srv_set_ref_;
  ros::ServiceServer srv_add_pivot_;
  ros::ServiceServer srv_add_straight_;
  ros::ServiceServer srv_calibrate_;

  std::string cam1_topic_;
  std::string cam2_topic_;
  double voxel_leaf_size_;
  int icp_max_iterations_;
  double icp_transformation_epsilon_;
  double icp_max_correspondence_distance_;
  int min_pivot_samples_;
  int min_straight_samples_;

  std::mutex mutex_;
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_1_;
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_2_;
  CloudT::Ptr ref_cloud_1_;
  CloudT::Ptr ref_cloud_2_;
  std::vector<PivotSample> pivot_samples_;
  std::vector<Eigen::Matrix4d> straight_samples_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_cam_full_calib");
  DualCamFullCalibNode node;
  node.init();
  ros::spin();
  return 0;
}
