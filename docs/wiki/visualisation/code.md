# 可视化（code）

> 模块：[src/figaroh/visualisation/](../../../src/figaroh/visualisation/) · 算法见 [algorithm](algorithm.md)

`visualisation` 包是本工程**唯一**的 meshcat 消费者（[Home](../Home.md) 将其定位为「meshcat 3D 可视化」），在 Pinocchio `MeshcatVisualizer`（`PMV`）之上扩展，提供通用图元（球/柱/盒）的添加与位姿设置，颜色用 `meshcat.geometry.Material`（单整数 RGB）编码。它与 [tools/robotvisualization](../tools/code.md) 是**两套不互通的可视化栈**——后者走 Gepetto `viewer.gui.addSphere/...` 接口，颜色用 RGBA 浮点列表，位姿用 XYZQUAT 元组，且面向 COM/力/关节轴等**领域专用**图元。两者 viewer 对象、颜色编码、位姿格式、图元 API 均不同，不可混用（对比见 [algorithm](algorithm.md)）。

---

## 文件清单

| 文件 | 角色 |
|------|------|
| [`__init__.py`](../../../src/figaroh/visualisation/__init__.py) | 包入口；重导出 `colors` 子模块与 `MeshcatVisualizer` |
| [`visualizer.py`](../../../src/figaroh/visualisation/visualizer.py) | `MeshcatVisualizer(PMV)` 类 + `materialFromColor` 适配器 |
| [`colors.py`](../../../src/figaroh/visualisation/colors.py) | `rgb2int`、`material` 工厂、9 颜色常量、`colormap` 字典 |

---

## 公共 API

| 符号 | 签名 / 说明 |
|------|------------|
| `MeshcatVisualizer`（继承 `pinocchio.visualize.MeshcatVisualizer`） | meshcat 可视化器扩展 |
| `.__init__(robot=None, model=None, collision_model=None, visual_model=None, url=None)` | 双模构造：`robot` 给定时取 `robot.model/collision_model/visual_model`；否则用显式三模型。`url` 连外部 meshcat server，`url="classical"` → `tcp://127.0.0.1:6000`。两者皆不给则建裸 `meshcat.Visualizer()`（仅图元、无模型）。 |
| `.addSphere(name, radius, color)` | 添球；`color` 必填 |
| `.addCylinder(name, length, radius, color=None)` | 添圆柱；`color` 默认 `None`（见 gotcha） |
| `.addBox(name, dims, color)` | 添盒；`color` 必填 |
| `.applyConfiguration(name, placement)` | 设位姿：接受 `pin.SE3` / `ndarray(7,)` XYZ-quat / list·tuple；**不接受 4×4 矩阵** |
| `.delete(name)` | 删对象（经 `__getitem__`） |
| `.__getitem__(name)` | `return self.viewer[name]` |
| `materialFromColor(color)`（定义于 `visualizer.py`，**未在包 `__init__` 重导出**） | 5 分支颜色适配器（见 [algorithm](algorithm.md)）；只能经 `figaroh.visualisation.visualizer.materialFromColor` 访问 |
| `colors.rgb2int(r,g,b)` | $256^{2}r+256\,g+b$，meshcat 单整数颜色 |
| `colors.material(color, transparent=False)` | 构造 `meshcat.geometry.Material` |
| `colors.{red,blue,green,yellow,magenta,cyan,white,black,grey}` | 9 预置常量 |
| `colors.colormap` | 名称→Material 字典（9 项） |

---

## 关键 gotcha

- **`materialFromColor(None)` 返回字符串而非 `Material`**（latent bug）：None 分支执行 `random.sample(list(colors.colormap),1)[0]`，而 `list(colormap)` 取的是字典**键**（如 `"red"`），故返回一个 colormap 键字符串。本应取 `.values()` 得到 Material。该字符串被原样塞给 `viewer[name].set_object(prim, "red")`，meshcat 期望 `Material`，渲染异常。
- **`addCylinder` 的 `color=None` 默认值不一致**：`addSphere`/`addBox` 的 `color` 必填无默认，唯独 `addCylinder` 默认 `None`——直接调用即落入上述 None bug。三个图元接口风格不统一。
- **`applyConfiguration` 不接受 4×4 矩阵**：仅有 `pin.SE3` 与 `ndarray(7,)`（XYZ-quat）两条归一分支；4×4 ndarray 落入 else 记日志并 `return False`。**成功返回 `None`、失败返回 `False`**——调用方若按布尔判断会误把成功（`None`）当失败。
- **`url="classical"` 魔法字符串**：被硬编码映射到 `tcp://127.0.0.1:6000`，仅此一个特殊值。
- **`white=rgb2int(250,250,250)`、`black=rgb2int(5,5,5)` 非纯色**：非 255/0，是工程视觉取舍而非纯白/纯黑。
- **`visualisation` 包无专属单测**：[`tests/unit/test_robotvisualization.py`](../../../tests/unit/test_robotvisualization.py) 测的是 [tools/robotvisualization](../tools/code.md)（Gepetto 栈），不覆盖本 meshcat 模块。
- **`materialFromColor` 未在包 `__init__` 重导出**：`__init__.py` 只导出 `colors` 与 `MeshcatVisualizer`；该适配器仅在 `visualizer` 子模块内，包顶层 `figaroh.visualisation.materialFromColor` 不可用。
- **构造器的裸 viewer 模式**：`robot` 与 `model` 都不给时，`__init__` 跳过 `PMV.__init__`/`initViewer`，直接建裸 `meshcat.Visualizer()`——此时无加载模型，仅能 `addSphere/...` 画图元，`display`/`loadModel` 等继承自 PMV 的方法不可用。

---

## 用法速览

```python
from figaroh.visualisation import MeshcatVisualizer, colors

viz = MeshcatVisualizer(url="classical")                       # 裸 viewer（无模型），仅画图元
viz.addSphere("world/p", 0.05, colors.red)                     # color 直接给 Material
viz.applyConfiguration("world/p", [0.1, 0.2, 0.3, 0, 0, 0, 1])  # list → XYZ-quat (7,) → 4×4
viz.delete("world/p")                                          # 经 __getitem__ 删除
```

`robot` 给定时优先走 PMV 路径（加载模型，可 `display(q)`）；仅画图元则用裸 viewer。颜色可传 `colors.red`（Material）、`"red"`（查 colormap）、`[1,0,0]`（[0,1] 浮点）、`None`（**触发上述 bug，勿用**）。

---

→ 算法设计见 [algorithm](algorithm.md)
