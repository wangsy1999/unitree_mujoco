# G1 与 Newton MPM 沙地

独立材料交互示例：G1 以站姿 PD 控制落在可变形沙层上，机器人由
Newton 1.7.0 的 `SolverMuJoCo` 求解，沙地由 `SolverImplicitMPM` 求解，
沙地接触冲量转换成机器人刚体的力和力矩，在下一帧反馈给机器人。
不需要启动 DDS 控制器或加载 ONNX 策略。

在仓库根目录运行。当前 conda `unitree` 已安装必要物理依赖：

```bash
conda activate unitree
# GPU 配置：1.2 × 0.8 × 0.12 米沙层，14400 粒子
python example/newton_sand/run_g1_sand.py --device cuda:0 --seconds 10

# CPU 小场景：0.4 × 0.4 × 0.12 米，1280 粒子
python example/newton_sand/run_g1_sand.py \
  --config example/newton_sand/g1_sand_cpu.json --device cpu --seconds 2

# 仅检查默认场景和粒子预算，不执行仿真
python example/newton_sand/run_g1_sand.py --check-config

# 无窗口运行
python example/newton_sand/run_g1_sand.py \
  --config example/newton_sand/g1_sand_cpu.json --device cpu --headless --seconds 0.15
```

新环境可安装 `simulate_python/requirements-newton.txt`。窗口复用 MuJoCo
查看器，无额外 GUI 依赖。默认把粒子上包络重建为连续沙面，使用约 1 厘米
的显示网格、空间平滑和细沙纹理，随 MPM 粒子更新，显示下陷与堆积。
显示网格不参与碰撞，不改变 MPM 网格或材料参数。

需要查看原始粒子时添加 `--sand-view particles`；该诊断模式受窗口几何数量
上限限制，只抽样显示部分沙粒并放大显示半径。全部粒子始终参与物理计算。
连续沙面是高度场近似，不能表现悬空沙团和翻卷结构；显示范围为初始沙层
边界向四周扩展 0.15 米，超出范围的飞散粒子不会在连续沙面中显示。
首次运行会编译 Warp 内核，耗时较长；CPU 不保证实时速度。

## 配置

编辑 `g1_sand.json` 或 `g1_sand_cpu.json`：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `bed_lower` / `bed_upper` | `[-0.6,-0.4,-0.12]` / `[0.6,0.4,0]` | 初始沙层边界，米 |
| `voxel_size` | 0.04 | MPM 网格尺寸，米；越小计算量越大 |
| `particles_per_cell` | 2 | 每个轴的采样密度；本例至少取 2 |
| `density` | 1600 | 用于计算粒子质量的初始体积密度，kg/m³ |
| `friction` | 0.65 | 沙材料摩擦参数 |
| `young_modulus` | 1000000 | 弹性模量，Pa |
| `frame_dt` / `rigid_substeps` | 0.01 / 5 | 每帧 MPM 一步，机器人五步，每步 0.002 秒 |
| `settle_seconds` | 0.1 | 固定初始机器人位姿时预沉降沙层的时间 |
| `spawn_height` | 0.81 | 初始 pelvis 高度，米 |

沙层底部对应 `scene_29dof_sand.xml` 中刚性底座的顶面 -0.12 米。
改变底部高度时要同步修改该 XML。沙层侧边没有刚性围墙，沙子可以向外铺开。
这些是演示参数，未按实测沙土标定。

## 适用范围与检查

耦合参考 Newton 官方的
[双向耦合示例](https://github.com/newton-physics/newton/blob/v1.7.0/newton/examples/mpm/example_mpm_twoway_coupling.py)。
采用滞后一帧的显式力反馈和刚体速度修正，对关节系统属于近似耦合，
不等于机器人与沙土的全耦合约束求解。减小时间步有助于控制显式耦合误差。

站姿 PD 只保持关节目标，不提供平衡控制；机器人可能倾倒，pelvis 低于
0.35 米时自动停止。本例没有给现有 parkour 网络提供沙面深度，也未接入 DDS；
可变形沙面的视觉观测和稳定行走控制需要另行实现。

已在 conda `unitree`、Newton 1.7.0、Warp 1.17.0、MuJoCo 3.15.0 上检查：
默认场景和 14400 粒子预算有效；CPU 缩小场景以 1280 粒子运行 0.15 秒，
观测到非零机器人沙地反作用力，单刚体峰值约 43 N，粒子高度范围
约 -0.116 到 -0.011 米，未穿透底座或出现非有限状态。
该短时检查关闭了预沉降，只验证接触与反馈链路，不证明长期稳定或物理精度。
未通过 Isaac Sim 验证；当前执行环境无可用 CUDA，GPU 和交互窗口未验证。
