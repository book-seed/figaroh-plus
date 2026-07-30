# measurements 算法页

> 模块：[src/figaroh/measurements/](../../../src/figaroh/measurements/) · 实现见 [code](code.md) · ⚠️ 遗留桩，未接入主管线

## 1. Tagged-union 测量表示

`Measurement` 用 `type` 字段做标签联合体，三类**互斥**：

| `type` | 值布局 | 构造 | pinocchio 类型 |
|--------|--------|------|---------------|
| `"SE3"` | `[x,y,z,roll,pitch,yaw]` | `pin.SE3(R, t)` | `pin.SE3` |
| `"wrench"` | 6D `[torque, force]` | `pin.Force(value)` | `pin.Force` |
| `"current"` | 原值 | 直存 | 原始类型 |

### 1.1 SE3：RPY → 旋转矩阵

`value[0:3] = [x,y,z]` 为平移 $\boldsymbol{t}$，`value[3:6] = [roll,pitch,yaw]` 经 `pin.rpyToMatrix([roll,pitch,yaw])` 得旋转矩阵 $R$，再 `pin.SE3(R, t)`。

Pinocchio 的 RPY 约定为 $R = R_z(\psi)\,R_y(\theta)\,R_x(\varphi)$（$\varphi$=roll, $\theta$=pitch, $\psi$=yaw）：

$$
R_x(\varphi)=\begin{bmatrix}1&0&0\\0&c_\varphi&-s_\varphi\\0&s_\varphi&c_\varphi\end{bmatrix},\quad
R_y(\theta)=\begin{bmatrix}c_\theta&0&s_\theta\\0&1&0\\-s_\theta&0&c_\theta\end{bmatrix},\quad
R_z(\psi)=\begin{bmatrix}c_\psi&-s_\psi&0\\s_\psi&c_\psi&0\\0&0&1\end{bmatrix}
$$

故

$$
\mathrm{SE3\_value} = \begin{bmatrix}R & \boldsymbol{t}\\0&1\end{bmatrix},\qquad
\boldsymbol{t}=[x,y,z]^\top
$$

### 1.2 wrench / current

- `wrench`：`pin.Force(value)` 按 pinocchio 约定为 6D $[\boldsymbol{\tau},\boldsymbol{f}]$（力矩在前，力在后）。
- `current`：原值直存，无额外封装。

## 2. add_SE3_measurement：frame 注入算法

把 SE3 测量作为一个**操作帧**（`pin.OP_FRAME`）挂到 pinocchio model 的指定关节 / 参考帧下：

```
function add_SE3_measurement(self, model):
    if self.type != "SE3":
        data = model.createData()        # 非 SE3 静默跳过 frame 注入
        return model, data
    parentJointId = model.getJointId(self.joint)       # 挂载关节
    parentFrameId = model.getFrameId(self.frame)       # 参考帧
    frame = pin.Frame(self.name, parentJointId, parentFrameId,
                      self.SE3_value, pin.OP_FRAME)
    self.frame_index = model.addFrame(frame, "False")  # ⚠️ "False" 疑似 bug
    data = model.createData()
    return model, data
```

返回 `(model, data)`：model 已附加新帧，data 为对应刷新的 `pin.Data`。

## 3. 当前状态：遗留桩

本模块**无实质运行算法**——如实说明：

- `Measurement` 类全仓库**零实例化**（`Measurement(` 仅命中类定义）。
- `add_SE3_measurement` **零调用**。`figaroh/__init__.py` 的 `from . import measurements` 只导入空包命名空间，`Measurement` 不可达。

真实测量数据流走两条**绕过本模块**的路径：

| 下游 | 数据来源 | 消费点 | 载体 |
|------|---------|--------|------|
| [calibration](../calibration/algorithm.md) | YAML `measurements` 配置段 | `config._extract_marker_info` / `_extract_poses` | plain `dict`（markers、poses） |
| [identification](../identification/algorithm.md) | 原始测量数组 | `tools.regressor.build_total_regressor_current` / `_wrench` | 原始 `numpy` 数组（`I_u/I_l`、`tau_u/tau_l`） |

- 标定侧：YAML 的 `markers`（`measurable_dof`）与 `poses`（`base_pose` / `tool_pose`）直接被 [config.py](../../../src/figaroh/calibration/config.py) 解析进 `calib_config` dict，不经过 `Measurement`。
- 辨识侧：关节电流 `I_u` / `I_l`、外力矩 `tau_u` / `tau_l` 以扁平 numpy 数组直接进入 [regressor.py](../../../src/figaroh/tools/regressor.py) 的总回归子拼接，不经过 `Measurement`。

**若要激活**本模块，需：(1) 在 `__init__.py` 重导出 `Measurement`；(2) 在上述数据加载处用 `Measurement(...)` 封装原始值并调用 `add_SE3_measurement` 注入帧。当前两者皆缺。
