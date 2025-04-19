# Trajectory.msg
std_msgs/Header header
time start_time                     # 轨迹起始时间
int32 traj_nums                     # 分段轨迹数量
float64 time_duration               # 轨迹总时间
float64[] poly_time                 # 每段轨迹的结束时间点
float64[] poly_coeff                # 多项式轨迹系数矩阵（展平为一维数组）
geometry_msgs/Point[] waypoints      # 路径点
geometry_msgs/Point hard_waypoints  # 硬路径点
int32 hard_waypoints_ID              # 硬路径点ID
float64 hard_waypoints_time          # 硬路径点时间
geometry_msgs/Point[] soft_waypoints_plist  # 软路径点列表
float64[] soft_waypoints_tlist       # 软路径点时间列表
bool replan_flag                     # 重规划标志

