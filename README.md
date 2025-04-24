# WorldWithoutAnime

基于Livox MID360激光雷达开发的无人机自主导航和避障软件。使用Rust开发，并由Bevy引擎进行可视化渲染。

![alt text](doc/example.png)

编写时截止至0fe0a40，v1.4-Soyo

## 使用说明

### 配置文件

格式如下，保证所有元素都正确赋值

``` toml
[hardware_config]
lidar_socket = "0.0.0.0:56301"
imu_socket = "0.0.0.0:56401"

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
p = 1       # never read
k = 0.5     # never read

[imu_config]
init_time = 1

[octree_config]
boundary = 2
max_depth = 5
voxel_size = 0.08

[apf_config]
k_att = 0.1
k_rep = 0.1
d0 = 0.5
epsilon = 0.3
step_size = 0.05
max_steps = 500

[l_shape_navigation_config]
d0 = 0.8
```

不建议调整除`l_shape_navigation_config`以外的值

### 编译

#### 编译为可执行文件

进入仓库根目录，运行`cargo build --release`

#### 编译为Python库

* 进入仓库根目录
* `conda activate MID360`
* `maturin develop --release`

### Python调用

启动`MID360` conda venv，示例如下：

``` Python
import world_without_anime
import time

config_path = r"H:\Project\Drones\src\WorldWithoutAnime\config.toml"    # 配置文件路径，没什么好说的，保险起见用绝对路径
#world_without_anime.run_mid360_with_bevy(config_path, False)

# 回调函数，Mavlink数据从这里获取
# data中即为MavlinkArgs，可以用迭代器访问
def data_received(data):
    for item in data:
        # 这里每个item为一个Mavlinkargs，用字段访问就行
        print(f"Velocity: {item.vx}, {item.vy}, {item.vz}")

world_without_anime.run_mid360_special_edition(config_path, data_received)  

# 防止Python进程退出
while (True):
    time.sleep(0.2)

# 该函数单独使用可进行可视化渲染，会主动持有线程，相当于无限循环
# 传入True为工训特调模式
world_without_anime.run_mid360_with_bevy(config_path, True)
```
