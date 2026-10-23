# 多后端仿真

G1 可变形沙地的独立演示见 [Newton MPM 沙地示例](../example/newton_sand/README.md)。

Python DDS 仿真和独立 parkour rollout 共用 `physics_backend.py` 的
`create_backend()` / `prepare()` / `step()` 接口。默认仍为 MuJoCo。

```bash
conda activate unitree
python simulate_python/unitree_mujoco.py --backend mujoco
python simulate_python/unitree_mujoco.py --backend newton --device cuda:0
```

也可以在 `config.py` 设置 `PHYSICS_BACKEND` 和 `NEWTON_DEVICE`。
原来的 C++ DDS 控制器、关节控制顺序和 parkour UDP 协议沿用现有接口。

Newton 依赖见 `requirements-newton.txt`，基于 Newton 1.7.0、Warp 1.17.0、
MuJoCo 3.15.0 和 MuJoCo Warp 3.15.0，匹配当前 `unitree` 环境的 MuJoCo。
Warp 固定为官方支持的 1.17.0，避免默认解析到对驱动要求更高的 1.18。
依赖已安装到 conda `unitree` 环境，重新安装可运行：

```bash
conda activate unitree
python -m pip install -r simulate_python/requirements-newton.txt
python simulate_python/unitree_mujoco.py --backend newton --device cpu
```

Newton 直接导入现有 MJCF，通过官方 `SolverMuJoCo` 步进：
`cuda:0` 使用 MuJoCo Warp；`cpu` 使用 Newton 的 MuJoCo CPU 参考路径。
CPU 路径适合接口诊断，不代表另一种独立物理求解算法。不会自动回退后端。
参考：[Newton SolverMuJoCo 官方源码](https://github.com/newton-physics/newton/blob/v1.7.0/newton/_src/solvers/mujoco/solver_mujoco.py)、
[官方依赖声明](https://github.com/newton-physics/newton/blob/v1.7.0/pyproject.toml)。

MuJoCo 的模型和数据作为主机侧兼容层，负责 DDS 传感器、深度射线及窗口；
Newton 步进后按名称映射状态回该兼容层，并调用 `mj_forward` 更新观测，
不会再调用主机的 `mj_step`。浮动基座转换包含四元数顺序、1.7 版父关节锚点
坐标系中的相对位姿和速度，以及质心/原点速度差；虚拟弹簧外力和窗口重置
在下一物理步同步到 Newton。
目前支持 free、hinge、slide 和 fixed 关节，遇到不支持的结构会报错。

这是单环境兼容接入，含每步 CPU/GPU 状态拷贝；没有进行批量环境或吞吐优化。
Newton 导入后的接触、惯量和传感器数值不保证与原始 MuJoCo 完全一致，
加速度/力传感器由主机侧 `mj_forward` 重算，不是 Newton 原生传感器。

独立 Python 推理需要另外安装 `example/parkour/requirements.txt` 中的
ONNX Runtime 等依赖（当前 `unitree` 未安装 ONNX Runtime）：

```bash
python example/parkour/run_mujoco.py --backend newton --device cpu \
  --headless --fixed-base --seconds 0.02
```

不通过 Isaac Sim 验证；短时接口检查不能证明自由运动或跑酷策略稳定性。

本次 1.7.0 检查直接使用 conda `unitree` 的 Python 3.10 和已安装依赖：
Newton CPU 的 5 步（0.01 秒）检查覆盖
非零电机输入、外力、旋转且运动的浮动基座及阻尼参数保留。当前执行环境
未检测到 CUDA，因此未验证 GPU、完整 DDS/窗口链路或策略自由运动。
