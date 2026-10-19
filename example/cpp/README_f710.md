# Logitech F710 控制 G1 仿真

保持当前部署结构：Python启动MuJoCo，C++执行策略并发送DDS命令。
F710由C++控制器直接读取Linux `/dev/input/js*`，Python的 `USE_JOYSTICK` 保持 `0`。
支持 `g1_parkour` 和DreamWaQ `g1_locomotion`。以下步骤以Parkour为例。

## 连接和输入检查

1. 将F710的X/D开关切到 **X（XInput）**，插入USB无线接收器并唤醒手柄。
   输入模式说明见 [Logitech官方文档](https://support.logi.com/hc/en-ch/articles/360023400073-Using-the-input-mode-switch-on-the-F710-to-select-DirectInput-or-XInput)。
2. 在运行控制器的Linux机器上查看设备：

   ```bash
   ls -l /dev/input/js* /dev/input/by-id/*joystick
   ```

   以下示例使用 `/dev/input/js0`，也可使用 `/dev/input/by-id/` 中对应的稳定设备链接。
   进程需要对设备有读取权限。若使用容器或远程开发环境，需要将USB输入设备提供给运行C++的环境。
3. 在仓库根目录编译：

   ```bash
   cmake -S example/cpp -B example/cpp/build \
     -DBUILD_G1_PARKOUR=ON \
     -DONNXRUNTIME_ROOT=/home/wsy/rl/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0
   cmake --build example/cpp/build --target g1_parkour g1_gamepad_test -j4
   ```

4. 先检查输入，不启动DDS、不加载网络、不发送命令：

   ```bash
   ./example/cpp/build/g1_parkour --joystick /dev/input/js0 --check-joystick
   ```

   左杆向前时 `forward` 应为正，右杆向左时 `yaw` 应为正；居中时两者为0。
   按住RB后 `command` 才随摇杆变化，松开RB后立即回到0。
   A、Y、Start、X应分别打印 `action=p/r/9/x`；B退出检查。
   若十字键和左摇杆功能互换，使用手柄MODE按钮恢复摇杆方向后重新检查。

   如果`forward`一直为0，先按MODE使模式灯熄灭；灯亮时F710会交换左杆与十字键，
   详见 [官方手册](https://www.logitech.com/assets/34885/3/f710620-002923003403gswamr.pdf)。
   新诊断输出包含每个轴的原始值：左杆前后应改变`0x1`（ABS_Y），
   若只改变`0x11`（ABS_HAT0Y），说明左杆信号在十字键通道。
   `forward`不依赖RB，只有`command`才受RB影响。

代码通过Linux轴/按钮映射查询 `ABS_Y`、`ABS_RX`、`BTN_A/B/X/Y` 等事件码，
不依赖固定的轴数组序号。XInput按钮事件码对应Linux
[xpad驱动](https://github.com/torvalds/linux/blob/master/drivers/input/joystick/xpad.c)。
D模式不作为支持目标；布局不具备所需轴和按钮时，程序会拒绝启动。
检测失败会分别报告ioctl读取错误，或列出缺失的轴/按钮及设备实际映射。
成功启动也会打印设备名称和映射，便于核对是否选中了正确的`js`设备。

## 启动仿真和控制器

```bash
# 终端1：原有Python仿真
/home/wsy/miniconda3/envs/unitree/bin/python simulate_python/unitree_mujoco.py
```

```bash
# 终端2：C++推理，开启F710输入
./example/cpp/build/g1_parkour --joystick /dev/input/js0
```

不带 `--joystick` 时维持原键盘控制。
启用手柄时，键盘仍可使能、复位、进入策略、清零和退出；速度由摇杆决定。
未识别到指定设备会直接报错，不自动切换到键盘速度控制。

| F710输入 | 行为 |
| --- | --- |
| Start | 使能，相当于键盘9；尚未收到有效仿真状态时不使能 |
| A | 平滑复位，相当于p；先按Start |
| Y | 复位完成后进入策略，相当于r；摇杆须居中、RB须松开 |
| 按住RB + 左摇杆前推 | 前向速度0～1m/s；后拉只归零，不倒走 |
| 按住RB + 右摇杆左右 | 转向速度±1rad/s；左转为正 |
| 松开RB | 三维速度清零，继续零命令策略 |
| X | 速度清零并锁住摇杆运动；松开RB并将摇杆居中后才能再次按RB运动 |
| B | 清零输出并退出控制器 |

按 `Start → A → 等待Holding training pose → Y` 进入策略。
刚进入时为零命令；之后按住RB操作摇杆。
摇杆有12%死区，死区外线性缩放。横向速度固定0，与训练设置一致。
虚拟挂带仍由MuJoCo窗口的7/8/9控制，手柄不切换挂带。

启动时已按住的按钮不触发使能、复位或策略启动，必须松开后重新按下。
USB设备断开或读取失败时，控制器停止并清零；无线失联是否表现为设备断开取决于驱动，
不能把USB断开检测当作无线链路心跳。

## 验证

```bash
ctest --test-dir example/cpp/build -R g1_gamepad --output-on-failure
```

模拟输入检查覆盖死区、摇杆正负方向、RB运动门控、初始化按钮抑制、按下沿、
按钮映射和B退出优先级。当前开发环境没有 `/dev/input`，尚未实测实体F710；
请在连接接收器的机器执行上面的只读输入检查。

整体部署、相机和控制说明见 [G1 Parkour README](README_g1_parkour.md)。

## DreamWaQ 用法

```bash
# 编译（同样需要配置ONNXRUNTIME_ROOT）
cmake -S example/cpp -B example/cpp/build -DBUILD_G1_LOCOMOTION=ON \
  -DONNXRUNTIME_ROOT=/home/wsy/rl/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0
cmake --build example/cpp/build --target g1_locomotion g1_gamepad_test -j4
# 只读输入检查
./example/cpp/build/g1_locomotion --joystick /dev/input/js0 --check-joystick
# 当前Python仿真启动后运行
./example/cpp/build/g1_locomotion --joystick /dev/input/js0
```

生命周期按钮与Parkour一致：Start使能、A复位、Y进入策略、X速度清零、B退出。
DreamWaQ支持后退和横向速度：按住RB，左杆前后控制前后速度，左杆左右控制横向速度，
右杆左右控制转向。速度按 `g1_locomotion.yaml` 的上下限缩放，松开RB归零。
三个运动轴都居中且RB松开时才允许进入策略；不使用Parkour的前向单向限制。
详细启动顺序见 [DreamWaQ说明](README_g1_locomotion.md)。
