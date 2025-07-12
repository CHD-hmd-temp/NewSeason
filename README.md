# NewSeason

WorldWithoutAnime的ROS1移植版

基于Livox MID360激光雷达开发的无人机自主导航和避障软件。使用Rust开发，并由Bevy引擎进行可视化渲染。

![alt text](doc/example.png)

编写时截止至2025-05-26，v1.4-Soyo

## 使用说明

### 配置文件

格式如下，保证所有元素都正确赋值

``` toml
[hardware_config]
lidar_socket = "0.0.0.0:56301"
imu_socket = "0.0.0.0:56401"

# 实验功能，未启用，请保留
[occupancy_map_config]
res = 0.05
width = 1000
height = 1000
origin = [0.0, 0.0]

[lidar_config]
dt = 100

[kalman_filter_config]
q = 0.01
r = 0.01
p = 1               # never read
k = 0.5             # never read

[imu_config]
init_time = 1

[octree_config]
boundary = 2        # 限制激光雷达数据读取范围
max_depth = 5       # 八叉树最大深度，结合bevy查看叶节点大小进行调整
voxel_size = 0.08   # 体素滤波网格大小，不得小于0.05m

[apf_config]
k_att = 0.1
k_rep = 0.1
d0 = 0.5            # 障碍物作用距离，超出阈值的障碍物贡献较低
epsilon = 0.3       # 路径终点与目标点的允许偏差，放大该值以生成更宽松的路径
step_size = 0.05    # 每步的步长，根据测试调整
max_steps = 500     # 最大步数

[l_shape_navigation_config]
d0 = 0.8            # 探测距离，用于状态机切换时的环境检查，单位为m，根据测试效果调整
```

根据测试结果调整除配置

### 编译

首次编译耗时较长，可能超过十分钟

#### 编译为可执行文件

进入仓库根目录，运行`cargo build --release`

#### 编译为Python库

要求Python >= 3.6，安装maturin

* 进入仓库根目录
* `conda activate <venv_name>`
* `maturin develop --release`

### Python调用

启动刚才安装的conda venv，示例如下：

``` Python
import world_without_anime
import time

config_path = r"H:\Project\Drones\src\WorldWithoutAnime\config.toml"
#world_without_anime.run_mid360_with_bevy(config_path, False)

# 回调函数，Mavlink数据从这里获取
# data中即为MavlinkArgs，可以用迭代器访问
def data_received(data):
    for item in data:
        # 这里每个item为一个Mavlinkargs，用字段访问就行
        print(f"Velocity: {item.vx}, {item.vy}, {item.vz}")

# 该函数执行后每五秒钟执行一次局部路径规划，无人机利用这段时间完成移动
# 每次发布的指令为List[MavlinkArgs]，元素数量可能超过100个
# 从第一个元素开始执行
# 自动避障功能也集成在函数中，自动避障全时间段开启，频率约为9Hz
# 触发自动避障时会发送只包含一个元素的List[MavlinkArgs]，应立即执行
world_without_anime.run_mid360_special_edition(config_path, data_received)

# 防止Python进程退出
while (True):
    time.sleep(0.2)

# 该函数单独使用可进行可视化渲染，会主动持有线程，相当于无限循环
# 使用时注意Python端线程安全
# 传入True为工训特调模式
world_without_anime.run_mid360_with_bevy(config_path, True)
```

### ROS话题订阅与发布

#### 订阅话题

在`/src/msg.rs`中手动包括需要解析的话题类型，话题路径应该位于`ROSRUST_MSG_PATH`环境变量中，详见rosrust

例如，订阅`/Odometry`话题（类型为`nav_msg/Odometry`），需要包含该路径

#### 发布话题

开发中
