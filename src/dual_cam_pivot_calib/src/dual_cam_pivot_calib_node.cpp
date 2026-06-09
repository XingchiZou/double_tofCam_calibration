/**
 * dual_cam_pivot_calib_node.cpp
 *
 * 双相机原地旋转标定节点：订阅两路点云，通过 ICP 估计相对基准帧的运动，
 * 再基于纯绕 Z 轴旋转约束求解水平力臂与外参。
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
#include <sstream>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class DualCamPivotCalibNode
{
public:
  DualCamPivotCalibNode()
    : nh_()
    , pnh_("~")
  {
  }

  void init()
  {
    loadParameters();

    sub_cam1_ = nh_.subscribe(cam1_topic_, 1, &DualCamPivotCalibNode::cam1Callback, this);
    sub_cam2_ = nh_.subscribe(cam2_topic_, 1, &DualCamPivotCalibNode::cam2Callback, this);

    srv_set_ref_ = pnh_.advertiseService("set_reference", &DualCamPivotCalibNode::setReference, this);
    srv_add_sample_ = pnh_.advertiseService("add_pivot_sample", &DualCamPivotCalibNode::addPivotSample, this);
    srv_calibrate_ = pnh_.advertiseService("calibrate", &DualCamPivotCalibNode::calibrate, this);

    ROS_INFO("dual_cam_pivot_calib initialized waiting for point clouds and service calls...");
  }

private:
  void loadParameters()
  {
    pnh_.param<std::string>("cam1_topic", cam1_topic_, "/cam1/points");
    pnh_.param<std::string>("cam2_topic", cam2_topic_, "/cam2/points");
    pnh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.05);
    pnh_.param<int>("icp_max_iterations", icp_max_iterations_, 50);
    pnh_.param<double>("icp_transformation_epsilon", icp_transformation_epsilon_, 1e-8);
    pnh_.param<double>("icp_max_correspondence_distance", icp_max_correspondence_distance_, 0.5);
    pnh_.param<int>("min_samples", min_samples_, 2);

    ROS_INFO("Parameters loaded:");
    ROS_INFO("  cam1_topic: %s", cam1_topic_.c_str());
    ROS_INFO("  cam2_topic: %s", cam2_topic_.c_str());
    ROS_INFO("  voxel_leaf_size: %.4f", voxel_leaf_size_);
    ROS_INFO("  icp_max_iterations: %d", icp_max_iterations_);
    ROS_INFO("  icp_transformation_epsilon: %.2e", icp_transformation_epsilon_);
    ROS_INFO("  icp_max_correspondence_distance: %.4f", icp_max_correspondence_distance_);
    ROS_INFO("  min_samples: %d", min_samples_);
  }

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
      ROS_WARN("ICP empty cloud after downsampling, src points: %zu, tgt points: %zu", src->size(), tgt->size());
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

  bool setReference(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!latest_cloud_1_ || !latest_cloud_2_)
    {
      res.success = false;
      res.message = "Point clouds from both cameras have not been received, cannot set reference frame";
      return true;
    }

    ref_cloud_1_ = toPcl(latest_cloud_1_);
    ref_cloud_2_ = toPcl(latest_cloud_2_);
    samples_.clear();

    res.success = true;
    res.message = "Reference frame updated";
    ROS_INFO("%s", res.message.c_str());
    return true;
  }

  bool addPivotSample(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!ref_cloud_1_ || !ref_cloud_2_)
    {
      res.success = false;
      res.message = "Please call set_reference first to set the reference frame";
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
      res.message = "ICP failed, sample not added to queue";
      return true;
    }

    samples_.push_back({A_k, B_k});

    std::ostringstream oss;
    oss << "Number of valid samples collected: " << samples_.size();
    res.success = true;
    res.message = oss.str();
    ROS_INFO("%s", res.message.c_str());
    return true;
  }

  Eigen::Vector2d solveHorizontalLeverArm(const std::vector<Eigen::Matrix4d>& transforms) const
  {
    const int N = static_cast<int>(transforms.size());
    Eigen::MatrixXd M(2 * N, 2);
    Eigen::VectorXd b(2 * N);

    for (int k = 0; k < N; ++k)
    {
      const Eigen::Matrix3d R = transforms[k].block<3, 3>(0, 0);
      const Eigen::Vector3d t = transforms[k].block<3, 1>(0, 3);
      const Eigen::Matrix2d R2 = (R.block<2, 2>(0, 0) - Eigen::Matrix2d::Identity());

      M.block<2, 2>(2 * k, 0) = R2;
      b(2 * k) = t.x();
      b(2 * k + 1) = t.y();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.solve(b);
  }

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

  bool calibrate(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (static_cast<int>(samples_.size()) < min_samples_)
    {
      res.success = false;
      std::ostringstream err;
      err << "Not enough samples, at least " << min_samples_ << " valid samples are required";
      res.message = err.str();
      return true;
    }

    std::vector<Eigen::Matrix4d> A_list;
    std::vector<Eigen::Matrix4d> B_list;
    for (const auto& sample : samples_)
    {
      A_list.push_back(sample.A);
      B_list.push_back(sample.B);
    }

    const Eigen::Vector2d r1 = solveHorizontalLeverArm(A_list);
    const Eigen::Vector2d r2 = solveHorizontalLeverArm(B_list);

    const Eigen::Quaterniond q_X = solveRotationAXEqualsXB(A_list, B_list);
    const Eigen::Matrix3d R_X = q_X.toRotationMatrix();
    const Eigen::Vector3d t_X = solveTranslation(A_list, B_list, R_X);

    Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
    X.block<3, 3>(0, 0) = R_X;
    X.block<3, 1>(0, 3) = t_X;

    std::ostringstream oss;
    oss << "\n========== calibration results ==========\n"
        << "Camera 1 horizontal lever arm: r1x=" << r1.x() << ", r1y=" << r1.y() << "\n"
        << "Camera 2 horizontal lever arm: r2x=" << r2.x() << ", r2y=" << r2.y() << "\n"
        << "Extrinsic matrix X (T_C1<-C2):\n"
        << X << "\n==============================";

    ROS_INFO("%s", oss.str().c_str());

    res.success = true;
    res.message = "Calibration completed, results have been output to the terminal";
    return true;
  }

  struct Sample
  {
    Eigen::Matrix4d A;
    Eigen::Matrix4d B;
  };

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_cam1_;
  ros::Subscriber sub_cam2_;
  ros::ServiceServer srv_set_ref_;
  ros::ServiceServer srv_add_sample_;
  ros::ServiceServer srv_calibrate_;

  std::string cam1_topic_;
  std::string cam2_topic_;
  double voxel_leaf_size_;
  int icp_max_iterations_;
  double icp_transformation_epsilon_;
  double icp_max_correspondence_distance_;
  int min_samples_;

  std::mutex mutex_;
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_1_;
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_2_;
  CloudT::Ptr ref_cloud_1_;
  CloudT::Ptr ref_cloud_2_;
  std::vector<Sample> samples_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_cam_pivot_calib");
  DualCamPivotCalibNode node;
  node.init();
  ros::spin();
  return 0;
}
