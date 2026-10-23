# G1 机载 DreamWaQ 部署

本入口将根目录 `policy.onnx` 对应的 DreamWaQ 部署到G1开发计算单元。
原生C++程序直接订阅机器人 `rt/lowstate` 并发布 `rt/lowcmd`，使用DDS domain 0；
无需启动MuJoCo、Python、UDP深度发送器或ROS。策略读取LowState主IMU，使用pelvis机体系。

构建入口复用已在MuJoCo验证的观测、关节映射和动作处理；实机模式启用已有的CRC、
型号、电机状态、超时、倾角、温度、目标变化率和估算力矩检查。
这些软件检查没有替代实机验收，目前没有连接真机验证行走稳定性。

## 机载位置与网络

Unitree官方开发资料区分运动控制计算机PC1（`192.168.123.161`）和开发计算机PC2
（`192.168.123.164`），参见 [官方网络说明](https://github.com/unitreerobotics/xr_teleoperate/wiki/CycloneDDS-and-UnitreeSDK-%28en%29)。
在开发计算机上安装和运行本程序，保留运动控制计算机的厂商系统。
实际交付地址、CPU架构和网卡名称以现场设备为准；程序不硬编码 `eth0`。

通过已有SSH或机载终端登录开发计算机，执行：

```bash
uname -m
ip -br address
ip route get 192.168.123.161
```

记录最后一条命令的 `dev` 网卡及 `src` 地址，例如 `dev eth0 src 192.168.123.164`。
后续使用这个网卡；机载运行也不使用 `lo`。不用修改开发电脑的网络来代替机载网卡配置。

## 准备独立源码包

可在开发电脑只打包本次DreamWaQ需要的C++源码、测试、配置和模型：

```bash
python3 deploy/g1/package_source.py
```

输出 `deploy/g1/build/g1-dreamwaq-source.tar.gz`，不包含MuJoCo、地形、Parkour或本机可执行文件。
通过已有文件传输方式将此包复制到开发计算机后解压：

```bash
tar -xzf g1-dreamwaq-source.tar.gz
cd g1-dreamwaq-source
```

随后按下一节原生编译。打包脚本只在准备源码时需要Python，控制程序运行不需要Python。

## 在机载计算机原生编译

先将当前仓库完整复制到开发计算机。安装CMake>=3.16、C++17编译器、
Unitree SDK2、Boost开发头文件、yaml-cpp开发包，以及与 `uname -m` 匹配的CPU版
ONNX Runtime C/C++发行包。SDK2安装步骤见 [官方SDK2 README](https://github.com/unitreerobotics/unitree_sdk2)。
SDK2及其CycloneDDS运行库需在机载系统中正确安装和注册。

- `aarch64` 使用Linux ARM64/AArch64版ORT和SDK。
- `x86_64` 使用Linux x64版ORT和SDK。
- 不能将当前x86开发机编译的可执行文件复制给ARM机载电脑。

在机载电脑的仓库根目录执行，ORT路径换成机载实际安装位置：

```bash
export ONNXRUNTIME_ROOT=/path/to/native/onnxruntime
cmake -S deploy/g1 -B deploy/g1/build -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT" \
  -DCMAKE_INSTALL_PREFIX="$PWD/deploy/g1/build/package"
cmake --build deploy/g1/build -j4
ctest --test-dir deploy/g1/build --output-on-failure
cmake --install deploy/g1/build
```

此CMake入口只构建DreamWaQ控制器及离线检查，不依赖MuJoCo或Parkour模型。
默认模型来自仓库根目录 `policy.onnx`，配置为 `example/cpp/g1_locomotion.yaml`。
模型训练参数及关节映射说明见仓库的 `example/cpp/README_g1_locomotion.md`。

安装目录结构：

```text
package/
  g1                         相对路径启动脚本
  bin/g1_locomotion           机载架构可执行文件
  config/g1_locomotion.yaml   默认角、PD、观测及实机保护参数
  models/policy.onnx          DreamWaQ网络
  lib/                       本次构建选定的ORT运行库
  licenses/onnxruntime/       ORT许可证和第三方声明
  BUILD_INFO.txt              架构、编译器与构建类型
  README.md
```

启动脚本显式传入包内模型和配置，整个package目录可在同一台机器上移动。
SDK2/CycloneDDS、yaml-cpp、C++和系统运行库仍由机载系统提供，不是完全静态程序。
检查依赖，输出中不应有 `not found`：

```bash
ldd deploy/g1/build/package/bin/g1_locomotion
cd deploy/g1/build/package
cat BUILD_INFO.txt
./g1 check
```

`check`只运行模型，不初始化DDS。`./g1`不带参数只显示帮助，不接管机器人。
不要配置开机自动启动或自动进入策略；本入口要求操作者显式接管。

需要打包安装结果时，在仓库根目录执行：

```bash
cd deploy/g1/build
cpack -G TGZ
```

压缩包名称包含构建CPU架构；仅适用于对应架构和兼容系统运行库的设备。

## 先只读检查

在安装包目录检查网络：

```bash
./g1 interfaces
# 将eth0换成上面路由检查得到的实际网卡
./g1 read-only eth0
```

只读入口只接收LowState，不初始化命令发布器，不释放原有运动控制服务，不加载模型。
检查CRC、`mode_machine=5`、29个关节状态和主IMU，成功后退出。
`mode_machine=5`是当前代码对29DOF机型的约束；若交付机型、腰部开放状态或固件不匹配，
先核对型号与接口，不通过修改常量强行绕过检查。

## 控制运行

先按该机器人官方流程进入低层开发状态；
[官方部署示例](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/README.md)
要求吊装并在零力矩状态进入调试模式。首次测试使用吊架和独立停止手段，确认现场人员已就绪。
模型、PD和关节边界须与实际G1硬件修订版核对，仿真站立通过不代表实机已经验收。

```bash
./g1 run eth0
```

启动时命令输出关闭。键盘操作：

1. `9`：释放原运动控制服务并确认接管，进入阻尼等待。
2. `p`：从实测角度平滑复位，等待 `Holding default pose`。
3. `r`：接近训练默认姿态后进入策略，初始速度为0。
4. `w/s`前后、`a/d`横移、`q/e`转向，`x`清零速度；`p`返回复位。
5. 空格或Ctrl+C停止。实机停止路径发送约0.5秒阻尼命令，不自动恢复官方运动模式。

机载部署不用仿真窗口的虚拟挂带按键。默认关节姿态及PD仍取自训练配置，未因打包而修改。

## F710（可选）

将接收器插入运行控制器的开发计算机，设置XInput，MODE灯熄灭：

```bash
./g1 joystick /dev/input/js0
./g1 run eth0 /dev/input/js0
```

输入检查不初始化DDS。实机控制映射：Start接管、A复位、Y进入策略，
按住RB接受摇杆速度；左杆前后/左右控制前后和横移，右杆左右控制转向。
松开RB速度归零；X清零并锁定运动，松开RB且摇杆居中后可重新运动；B停止并退出。
设备断开或读取失败会进入同一阻尼停止路径。

F710由Linux设备直接读取，不写入机器人 `wireless_remote`。
机器人原配遥控器A按钮在当前程序中始终是阻尼停止请求，与F710的A复位是两个独立来源；
其字节定义依据 [SDK官方遥控数据布局](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/gamepad.hpp)。
这不是对机器人固件硬件急停功能的声明。
无线失联不一定导致USB设备消失，本程序没有F710无线链路心跳，不能保证无线断链检测。

当前只完成本机构建、离线检查和MuJoCo回归。实际机载SDK兼容性、F710、
MotionSwitcher响应、控制板停止行为与行走稳定性仍需现场验证。
完整保护实现和限制见仓库 `example/cpp/README_g1_hardware.md`。
