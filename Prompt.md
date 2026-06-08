> **角色设定**
> 你是一位精通多视几何、PCL (Point Cloud Library) 点云处理以及 ROS 1 (Noetic) 架构的资深 C++ 机器人工程师。请帮我编写一个端到端的 ROS 功能包（命名为 `dual_cam_pivot_calib`），直接输入两个相机的原始点云，利用“刚体纯原地旋转”的动作，在线标定两个相机到物理旋转中心的水平力臂，以及它们之间的相对外参。
> **系统环境**
> * OS: Ubuntu 20.04
> * ROS: Noetic
> * Language: C++14
> * Dependencies: `roscpp`, `sensor_msgs`, `pcl_ros`, `pcl_conversions`, `Eigen3`, `std_srvs`
> 
> 
> **架构设计：点云配准 + 纯旋转数学标定**
> 本节点订阅两个相机的原始点云，在内部缓存最新帧。用户控制刚体进行**多次纯原地旋转（无平移）**，每次转动后调用服务提取样本，内部使用 PCL (ICP) 计算运动变换，最后解算力臂与外参。
> **ROS 节点接口定义**
> * **点云订阅**:
> * `/cam1/points` (sensor_msgs::PointCloud2)
> * `/cam2/points` (sensor_msgs::PointCloud2)
> 
> 
> * **服务 `~/set_reference` (Type: `std_srvs/Trigger`)**:
> * 调用时，将当前最新的 Cam1 和 Cam2 点云深拷贝并保存为 `ref_cloud_1` 和 `ref_cloud_2`（基准帧）。返回提示“基准参考帧已更新”。
> 
> 
> * **服务 `~/add_pivot_sample` (Type: `std_srvs/Trigger`)**:
> * 调用时，取出最新缓存的点云，分别与对应的 `ref_cloud_i` 执行 ICP 算法 (`pcl::IterativeClosestPoint`)。
> * ICP 的输出矩阵即为相对于基准帧的运动矩阵 $A_k$ (Cam1) 和 $B_k$ (Cam2)。将其转换为 `Eigen::Matrix4d` 存入观测队列。返回当前采集的有效样本总数。
> 
> 
> * **服务 `~/calibrate` (Type: `std_srvs/Trigger`)**:
> * 调用时，检查队列样本数（需 $N \ge 2$），提取数据执行标定核心算法。
> 
> 
> **数学标定原理与算法要求（非常重要，请严格实现）**
> 1. **ICP 预处理要求**：ICP 前必须添加 VoxelGrid 滤波（叶子大小 0.05m）以加速配准。如果 `hasConverged()` 为 false，打印 Warn 并丢弃该样本。
> 2. **水平力臂求解（核心算法 1）**：
> * 物理约束：刚体发生的是纯绕 Z 轴旋转，相机产生的平移是由力臂引起的。方程为 $\tilde{\mathbf{t}}_i = (\tilde{R}_i - I)\mathbf{r}_i$。
> * **降维防奇异处理**：由于 Z 轴无平移位移，不能直接求 3x3 的伪逆。针对相机 1，提取所有 $N$ 个样本的 $(R_k - I)$ 的**前 2 行前 2 列**，拼接成 $2N \times 2$ 的矩阵 $\mathbf{M}$。提取 $\mathbf{t}_k$ 的 $x, y$ 分量拼接成 $2N \times 1$ 的向量 $\mathbf{b}$。
> * 利用 `Eigen::JacobiSVD<Eigen::MatrixXd>(M, Eigen::ComputeThinU | Eigen::ComputeThinV)` 求解二维水平力臂 $\mathbf{r}_1 = [r_{1x}, r_{1y}]^T$。
> * 对相机 2 执行完全相同的操作求解 $\mathbf{r}_2 = [r_{2x}, r_{2y}]^T$。
> 
> 
> 3. **外参求解（核心算法 2）**：
> * 利用 $A_k X = X B_k$，解算相机 2 到相机 1 的外参 $X = T_{C1 \leftarrow C2}$。
> * 先通过 $R_A R_X = R_X R_B$ 解出旋转 $R_X$。
> * 再利用 $(R_{Ak} - I)\mathbf{t}_X = R_X \mathbf{t}_{Bk} - \mathbf{t}_{Ak}$ 构造最小二乘求解平移 $\mathbf{t}_X$。
> 
> 
> 4. **结果输出**：标定成功后，在 ROS_INFO 终端高亮打印解算出的 $(r_{1x}, r_{1y})$、$(r_{2x}, r_{2y})$ 以及外参矩阵 $X$。
> 
> 
> **交付物要求**
> 请提供以下完整代码文件内容：
> 1. `CMakeLists.txt`（包含 PCL 和 Eigen 链接配置）
> 2. `package.xml`
> 3. `dual_cam_pivot_calib_node.cpp`：请将所有的 ROS 回调、PCL 配准流水线和基于 Eigen 的 SVD 标定数学推导写在一个规范的 C++ 类中，并附带详尽的中文注释。