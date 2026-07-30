# 可视化（algorithm）

> 模块：[src/figaroh/visualisation/](../../../src/figaroh/visualisation/) · 实现见 [code](code.md)

本模块是工具层，**无复杂算法**——核心仅是颜色编码、颜色适配、位姿归一三件小事，以及与 Gepetto 栈的接口对比。本页诚实保持简短。

---

## 1. 颜色编码：rgb2int

meshcat 把 RGB 三通道打包成**单整数**而非 `(r,g,b)` 元组。编码函数：

$$
\text{rgb2int}(r,g,b) = 256^{2}\,r + 256\,g + b
$$

即把 $r$ 放到「65536 位」、$g$ 放到「256 位」、$b$ 放到「1 位」，等价于十六进制 $0xRRGGBB$（如纯红 $(255,0,0)\to 0xFF0000=16711680$）。之所以用单整数：meshcat 的 `Material.color` 是单一标量字段，前端按位拆回三通道渲染，比传数组省一次序列化。`colors.material(color, transparent=False)` 据此构造 `meshcat.geometry.Material`（设 `color`/`transparent`）。9 预置常量（`red..grey`）即由 `rgb2int` 算出；`white=rgb2int(250,250,250)`、`black=rgb2int(5,5,5)` 刻意非纯。

---

## 2. materialFromColor 适配器（5 分支）

把异质颜色输入归一为 `Material`：

```
function materialFromColor(color):
    if color is Material:        return color              # 原样
    elif color is str:          return colormap[color]    # 查名称表
    elif color is list:
        m = Material()
        m.color = rgb2int(*[int(c*255) for c in color[:3]])  # [0,1]浮点→[0,255]整
        if len(color)==3: m.transparent = False
        else: m.transparent = (color[3] < 1); m.opacity = float(color[3])  # RGBA
        return m
    elif color is None:
        return random.sample(list(colormap),1)[0]   # ⚠️ BUG: 取的是 dict 键(str), 非 Material
    else:                        return black       # 兜底
```

**None 分支 bug**：`list(colormap)` 迭代的是字典**键**（`["red","blue",...]`），故返回一个字符串键（如 `"red"`），而非 `Material`。正确写法应 `list(colormap.values())`。该字符串被原样塞给 `viewer[name].set_object(prim, "red")`，meshcat 期望 `Material`，渲染异常。详见 [code](code.md) gotcha。

---

## 3. applyConfiguration 位姿归一

输入多种位姿表示，归一为 4×4 齐次变换 $T$ 后交给 `viewer[name].set_transform(T)`：

$$
T = \begin{bmatrix} R & p \\ 0\,0\,0 & 1 \end{bmatrix}\in\mathbb{R}^{4\times4},\quad R\in SO(3),\ p\in\mathbb{R}^{3}
$$

- **`pin.SE3`** → 直接取 $R=$ `placement.rotation`、$p=$ `placement.translation`，拼成 $T$。
- **`ndarray(7,)` XYZ-quat** → 前 3 为平移 $p$，后 4 为四元数 $q=(x,y,z,w)$；由 `pin.Quaternion(q).matrix()` 得 $R$，再拼 $T$。
- **list/tuple** → 先 `np.array` 归一，再走上述。
- **其它（含 4×4 矩阵）** → 记日志、`return False`。

```
function applyConfiguration(name, placement):
    if list/tuple: placement = np.array(placement)
    if placement is pin.SE3:
        R = placement.rotation ; p = placement.translation
    elif placement is ndarray and placement.shape==(7,):
        q = placement[3:]                  # (x,y,z,w)
        R = pin.Quaternion(q.reshape(4,1)).matrix() ; p = placement[:3]
    else:
        log.error(); return False
    T = [[R, p], [0,0,0,1]]                # np.r_[np.c_[R,p], [[0,0,0,1]]]
    viewer[name].set_transform(T)           # 成功隐式 return None
```

注意成功返回 `None`、失败返回 `False`（见 [code](code.md) gotcha）；4×4 矩阵无专属分支，会被拒收。

---

## 4. 两套可视化栈对比

| | meshcat 栈（本模块） | Gepetto 栈（[tools/robotvisualization](../tools/algorithm.md)） |
|---|---|---|
| 对象接口 | `viewer[name].set_object(prim, Material)` / `set_transform(T)` | `viewer.gui.addSphere(name,r,RGBA)` / `addBox`/`addArrow`/`addXYZaxis` |
| 颜色 | `Material`，单整数 `rgb2int` | RGBA 浮点列表 `[r,g,b,a]` |
| 位姿 | 4×4 齐次矩阵 $T$ | XYZQUAT 7 元组（`pin.SE3ToXYZQUATtuple`） |
| 图元 | 通用 Sphere/Cylinder/Box | 领域专用 COM/force/joint axes/bbox + 球/盒 |
| 刷新 | 自动 | 需 `viewer.gui.refresh()` |

**为何不互通**：viewer 对象的 API 表面不同（`viewer[name]` vs `viewer.gui.addXxx`），颜色编码不同（单整数 `Material` vs RGBA 列表），位姿格式不同（4×4 矩阵 vs XYZQUAT 元组），图元集合不同（通用 vs 领域专用）。一个 meshcat `MeshcatVisualizer` 的 `viewer` 没有 `.gui.addSphere`；反过来 Gepetto viewer 无 `viewer[name].set_object`。故两栈各自独立、不可混用。
