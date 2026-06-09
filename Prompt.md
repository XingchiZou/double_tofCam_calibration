> **角色设定**
> 你是一位精通多视几何、PCL (Point Cloud Library) 点云处理以及 ROS 1 (Noetic) 架构的资深 C++ 机器人工程师。请帮我编写一个端到端的 ROS 标定功能包（命名为 `dual_cam_full_calib`）。
> **核心任务**：输入两个相机的原始点云，利用“刚体纯原地旋转”和“纯笔直前行”两种动作，在线计算机器人的全套视觉运动学参数。
> **系统环境**
> * OS: Ubuntu 20.04 | ROS: Noetic | Language: C++14
> * Dependencies: `roscpp`, `sensor_msgs`, `pcl_ros`, `pcl_conversions`, `Eigen3`, `std_srvs`
> 
> 
> **ROS 节点接口定义**
> * **点云订阅**: `/cam1/points`, `/cam2/points` (需内部缓存最新帧)。
> * **服务 `~/set_reference` (std_srvs::Trigger)**: 缓存当前两路点云为基准帧 `ref_cloud_1` 和 `ref_cloud_2`。
> * **服务 `~/add_pivot_sample` (std_srvs::Trigger)**: 当前点云与基准帧做 ICP (`pcl::IterativeClosestPoint` + VoxelGrid叶子0.05m)。输出的运动矩阵存入**旋转观测队列** (Cam1 的 $A_k$ 序列和 Cam2 的 $B_k$ 序列)。
> * **服务 `~/add_straight_sample` (std_srvs::Trigger)**: 当前点云与基准帧做 ICP。仅取 Cam1 的输出运动矩阵，存入**直行观测队列** ($T_{straight}$)。
> * **服务 `~/calibrate` (std_srvs::Trigger)**: 检查旋转队列 ($N \ge 2$) 和直行队列 ($M \ge 1$)，执行下述核心标定算法。
> 
> 
> **数学标定原理与算法流（非常重要，请严格按此顺序实现）**
> **阶段一：计算姿态纠正矩阵 $R_{align}$（消除机械安装误差）**
> * 物理背景：相机 $-X$ 轴理应指向刚体旋转 $Z$ 轴，求 $R_{align}$ 将真实转轴对齐到目标轴 $\mathbf{v}_{target} = [-1, 0, 0]^T$。
> * 对 Cam1 的所有原始旋转矩阵 $R_{Ak\_raw}$：
> 1. 用 `AngleAxisd` 提取出每次的旋转轴向量 $\mathbf{u}_k$。
> 2. 对所有 $\mathbf{u}_k$ 求和并归一化，得到平均真实转轴 $\mathbf{u}_{true}$。
> 3. 方向检查：若 $\mathbf{u}_{true} \cdot \mathbf{v}_{target} < 0$，则 $\mathbf{u}_{true} = -\mathbf{u}_{true}$。
> 4. 纠正矩阵：`R_align1 = Eigen::Quaterniond::FromTwoVectors(u_true, v_target).toRotationMatrix()`。
> 
> 
> * 对 Cam2 同理求出 $R_{align2}$。
> 
> 
> **阶段二：修正观测轨迹数据**
> * 对**旋转队列**中所有的矩阵进行坐标系修正：
> $R_{Ak} = R_{align1} \cdot R_{Ak\_raw} \cdot R_{align1}^T$
> $\mathbf{t}_{Ak} = R_{align1} \cdot \mathbf{t}_{Ak\_raw}$
> *(Cam2 用 $R_{align2}$ 修正)*。
> * 后续所有的计算必须使用修正后的矩阵！
> 
> 
> **阶段三：水平力臂求解（降维防奇异）**
> * 物理约束：相机平移方程 $\mathbf{t}_i = (R_i - I)\mathbf{r}_i$。
> * 对 Cam1 修正后的 $N$ 个旋转样本，取 $(R_{Ak} - I)$ 的**前 2 行前 2 列**拼成 $2N \times 2$ 矩阵 $\mathbf{M}$，取 $\mathbf{t}_{Ak}$ 的 $x, y$ 分量拼成 $2N \times 1$ 向量 $\mathbf{b}$。
> * 用 `Eigen::JacobiSVD` (带 ComputeThinU|V) 求解超定方程，得二维力臂 $\mathbf{r}_1 = [r_{1x}, r_{1y}]^T$。对 Cam2 同理求解 $\mathbf{r}_2$。
> 
> 
> **阶段四：外参求解 ($AX=XB$)**
> * 利用修正后的旋转序列 $A_k X = X B_k$，解算外参 $X = T_{C1 \leftarrow C2}$。
> * 先用 $R_{Ak} R_X = R_X R_{Bk}$ 解出旋转 $R_X$。再用 $(R_{Ak} - I)\mathbf{t}_X = R_X \mathbf{t}_{Bk} - \mathbf{t}_{Ak}$ 解出平移 $\mathbf{t}_X$。
> 
> 
> **阶段五：偏航纠正角 $\psi$ 求解（刚体坐标系对齐）**
> * 提取**直行队列**中的样本 $T_{straight\_raw}$。
> * 必须先施加纠正矩阵：$\mathbf{t}_{straight\_corr} = R_{align1} \cdot \mathbf{t}_{straight\_raw}$。
> * 提取修正后的水平平移分量（向左为 $y$，向前为 $z$）。
> * 计算偏角：$\psi = \text{atan2}(t_{straight\_corr}.y(), t_{straight\_corr}.z())$。
> 
> 
> **阶段六：终端输出**
> * 标定成功后，在 ROS_INFO 高亮打印所有结果：$R_{align1}$, $R_{align2}$, 物理力臂 $(r_{1x}, r_{1y})$, $(r_{2x}, r_{2y})$, 相对外参 $X_{12}$, 以及安装偏角 $\psi$ (转换为 Degree 显示)。
> 
> 
> **【行为与输出限制】（严格遵守）**
> 1. 只需编写所请求功能的核心源代码（`CMakeLists.txt`, `package.xml`, `dual_cam_full_calib_node.cpp`，`config.yaml`）。
> 2. 请勿创建任何 Dockerfile、CI/CD 配置或本地开发环境。
> 3. 请勿自动运行或测试代码。
> 4. 生成原始代码后，请立即停止操作并征得我的同意，再进行后续讨论。