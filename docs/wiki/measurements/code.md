# measurements 代码页

> 模块：[src/figaroh/measurements/](../../../src/figaroh/measurements/) · 算法见 [algorithm](algorithm.md) · ⚠️ 遗留桩，未接入主管线

## 模块定位

测量**表示层**：把一次测量（位姿 / 力矩 / 电流）封装成 `Measurement` 对象，并提供把它作为操作帧注入 pinocchio model 的接口。当前为**遗留桩 / 死代码**——类从未被实例化，方法从未被调用，真实测量数据走两条绕过本模块的路径（见 [algorithm](algorithm.md)）。

## 文件清单

| 文件 | 职责 |
|------|------|
| [measurement.py](../../../src/figaroh/measurements/measurement.py) | 定义 `Measurement` 类与 `add_SE3_measurement` |
| [__init__.py](../../../src/figaroh/measurements/__init__.py) | 仅 License 头，**无任何重导出** |

## 公共 API

| API | 签名 | 说明 |
|-----|------|------|
| `Measurement.__init__` | `(name, joint, frame, type, value)` — 全位置参数，**无默认值** | 按 `type` 分支构造测量对象 |
| `Measurement.add_SE3_measurement` | `(model) -> (model, data)` | 把 SE3 测量作为 `OP_FRAME` 注入 pinocchio model |

`type` 取值：`"SE3"` / `"wrench"` / `"current"`（裸字符串，非 Enum）。

## 实例属性

| 属性 | 类型 | 设置条件 |
|------|------|---------|
| `name` | str | 总是 |
| `joint` | str | 总是（挂载关节名） |
| `frame` | str | 总是（最近参考帧名） |
| `type` | str | 总是 |
| `SE3_value` | `pin.SE3` | 仅 `type == "SE3"` |
| `wrench_value` | `pin.Force` | 仅 `type == "wrench"` |
| `current_value` | 原值 | 仅 `type == "current"` |
| `frame_index` | int | 仅调用 `add_SE3_measurement` 且 `type == "SE3"` 后 |

## 关键 gotcha

- **零实例化**：全仓库 `Measurement(` 仅命中类定义本身；`add_SE3_measurement` 零调用。`figaroh/__init__.py` 的 `from . import measurements` 只导入空包命名空间，`Measurement` 不可达。
- **不重导出**：`__init__.py` 无内容，须 `from figaroh.measurements.measurement import Measurement`，但无任何代码这么做。
- **`addFrame(..., "False")` 疑似 bug**：`model.addFrame(frame, "False")` 第二参数传字符串 `"False"` 而非布尔 `False`；pinocchio `addFrame(frame, backend=None)` 期望 backend 标识符。
- **非 SE3 静默跳过**：`add_SE3_measurement` 对 `wrench` / `current` 不注入 frame，但仍 `createData()` 并返回，`frame_index` 不被设置。
- **无 Enum**：`type` 裸字符串匹配，拼写错误只能到 `else` 分支抛 `TypeError`。
- **无时间语义**：单次测量一个值，不支持时间戳 / 采样率 / 测量协方差加权。
- **原始 `value` 不保留**：构造后只存类型专属的 `SE3_value` / `wrench_value` / `current_value`，输入 6D 数组本身被丢弃，无法回查。

→ 算法设计见 [algorithm](algorithm.md)
