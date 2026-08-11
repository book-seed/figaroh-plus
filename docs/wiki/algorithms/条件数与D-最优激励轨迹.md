# 条件数与 D-最优激励轨迹

在机器人动力学参数辨识中，寻找最优激励轨迹（Optimal Excitation Trajectory）是提高参数估计抗噪性与精度的关键环节。评价回归矩阵 $\mathbf{W}$ 或费雪信息阵（Fisher Information Matrix, FIM） $\mathbf{F} = \mathbf{W}^T\mathbf{W}$ 质量的核心指标主要有两个：**条件数（Condition Number）** 与 **D-最优准则（D-optimal Criterion）**。

而连接这两个数学指标与实际辨识精度的桥梁，就是**置信椭球（Confidence Ellipsoid）**。

---

## 1. 置信椭球（Confidence Ellipsoid）的概念与物理本质

### 1.1 从一维区间到高维超椭球

* **一维参数（如测量关节 1 的质量 $m_1$）**：估计结果的不确定性用置信区间（区间线段）表示，例如 $[1.2\text{kg}, 1.4\text{kg}]$（95% 置信度）。
* **二维参数（如同时测量 $m_1$ 与转动惯量 $I_1$）**：两个参数存在联合测量不确定性与相关性，在二维平面上表现为一个**置信椭圆（Confidence Ellipse）**。
* **高维参数（如机器人 $p$ 个关节基参数）**：所有待估参数联合误差在高维空间中构成的超曲面，被称为 **$p$ 维置信超椭球（Confidence Ellipsoid）**。

### 1.2 物理本质：多维参数的“联合误差棒（Error Bar）”

**严格来说，置信椭球表征的就是“多维估计参数的误差（不确定性）分布范围”。**

可以将其直观理解为高维参数空间中的“联合误差棒”。具体表征三个维度的物理含义：

1. **表征的误差类型**
   
   在真实辨识中，传感器噪声会导致估计值 $\hat{\boldsymbol{\theta}}$ 与真实物理值 $\boldsymbol{\theta}^*$ 之间产生偏差 $\tilde{\boldsymbol{\theta}} = \hat{\boldsymbol{\theta}} - \boldsymbol{\theta}^*$。置信椭球圈出的范围，代表在指定置信度下（如 95%），真实参数可能出现的**误差分布极限**。

2. **为什么是“椭球”而非“方框”？**
   
   如果各参数的误差完全独立，误差边界将是一个正方体/方框。但机器人运动中参数间存在严重的**耦合与相关性**（例如关节 2 的加速同时影响关节 2 和关节 3 的受力），**一个参数的估计误差会牵连另一个参数**，因此联合误差分布在空间中呈现为倾斜的椭圆或椭球。椭球的倾斜姿态直接反映了参数误差之间的相关程度。

3. **椭球形态与误差状态的对应关系**
   
   **椭球总体积** $\rightarrow$ **总体不确定性（综合误差大小）**：体积越小，整体参数估得越准。

   **某个主轴的长度** $\rightarrow$ **该主成分方向上的误差大小**：某个轴越长，说明沿着该物理方向的参数**估计极不精确**，传感器噪声在该方向被严重放大。
   
   **长短轴比例（偏心率）** $\rightarrow$ **各方向误差的均衡程度**：椭球越扁，说明“部分参数测得很准，但另一些参数测得极差”。

### 1.3 数学定义与方程

假设利用传感器数据（如力矩与运动学数据）估计出 $p$ 个待求参数 $\hat{\boldsymbol{\theta}} \in \mathbb{R}^p$，且估计偏差服从多元高斯分布：

$$\hat{\boldsymbol{\theta}} \sim \mathcal{N}(\boldsymbol{\theta}^*, \mathbf{\Sigma})$$

其中 $\boldsymbol{\theta}^*$ 为真实参数，$\mathbf{\Sigma} \in \mathbb{R}^{p \times p}$ 为参数估计的**协方差矩阵（Covariance Matrix）**。在普通最小二乘法（OLS）中：

$$\mathbf{\Sigma} = \sigma^2 (\mathbf{W}^T \mathbf{W})^{-1} = \sigma^2 \mathbf{F}^{-1}$$

定义置信水平为 $1 - \alpha$（如 95%）的置信椭球方程为如下二次型不等式：

$$(\hat{\boldsymbol{\theta}} - \boldsymbol{\theta}^*)^T \mathbf{\Sigma}^{-1} (\hat{\boldsymbol{\theta}} - \boldsymbol{\theta}^*) \le c^2$$

式中：

* $\mathbf{\Sigma}^{-1} = \frac{1}{\sigma^2} \mathbf{F} = \frac{1}{\sigma^2} \mathbf{W}^T\mathbf{W}$，即**置信椭球的二次型系数矩阵正比于费雪信息阵**。
* $c^2 = \chi^2_p(1-\alpha)$ 为自由度为 $p$ 的卡方分布在指定置信度下的临界值。

### 1.4 置信椭球的三大几何要素

置信椭球在几何上的物理形态完全由协方差矩阵 $\mathbf{\Sigma}$（或费雪信息阵 $\mathbf{F}$）的特征值与特征向量决定：

```
                    z (特征向量 v3)
                    ^
                    │     .---.   <--- 轴长由 √λ3 决定
                    │   /       \
                    │  |    *    |  <--- 椭球中心 (估计均值 θ̂)
                    │   \       /
                    │     '---'
                    └───────────────────> y (特征向量 v2)
                   /
                  /  <--- 轴长由 √λ1, √λ2 决定
                 v x (特征向量 v1)

```

1. **椭球中心**：参数估计的均值 $\hat{\boldsymbol{\theta}}$。
2. **主轴方向（Orientation）**：由矩阵的特征向量（Eigenvectors）决定，代表参数误差的相关主成分方向。
3. **主轴长度（Axis Lengths）**：由协方差矩阵 $\mathbf{\Sigma}$ 的**特征值的平方根 $\sqrt{\lambda_i(\mathbf{\Sigma})}$** 决定（反比于 $\sqrt{\lambda_i(\mathbf{F})}$）。
* 特征值 $\lambda_i(\mathbf{\Sigma})$ 越大 $\Rightarrow$ 该方向的不确定性越高 $\Rightarrow$ **椭球在该轴向上越长**。
* 特征值 $\lambda_i(\mathbf{\Sigma})$ 越小 $\Rightarrow$ 该方向的估计精度越高 $\Rightarrow$ **椭球在该轴向上越短**。



---

## 2. 几何直观：D-最优与条件数在控制什么？

在轨迹优化中，调节机器人运动轨迹（即改变回归矩阵 $\mathbf{W}$）的本质，就是**通过优化轨迹去“捏塑”这个置信椭球的形态**：

```
          y (参数 2 误差)
             ^
             │      /────────────────\
             │     /                  \   <--- 置信椭圆 (Uncertainty Ellipsoid)
             │    /      *(真值)       \
             │   \                      /
             │    \                    /
             │     \──────────────────/
             └────────────────────────────> x (参数 1 误差)

```

* **D-optimal（D-最优）**：控制 **“椭球的总体积”**。
其核心目标是不惜一切代价将置信椭球的超体积压缩到最小。总体积越小，代表综合所有维度看，参数估计的整体不确定性越低。
* **条件数（Condition Number）**：控制 **“椭球的形状（偏心率）”**。
其核心目标是把置信椭球调得尽可能接近“正球体”（最长轴与最短轴之比接近 1），避免出现“某些维度极度精确，但另一些维度极度模糊”的病态倾斜。

---

## 3. 数理根基：统计学与矩阵理论

设机器人动力学辨识的线性回归模型为：

$$\boldsymbol{\tau} = \mathbf{W} \boldsymbol{\theta} + \boldsymbol{\epsilon}, \quad \boldsymbol{\epsilon} \sim \mathcal{N}(\mathbf{0}, \sigma^2 \mathbf{I})$$

其中 $\boldsymbol{\tau} \in \mathbb{R}^{Nn}$ 为采样到的关节力矩，$\mathbf{W} \in \mathbb{R}^{Nn \times p}$ 为回归矩阵，$\boldsymbol{\theta} \in \mathbb{R}^p$ 为待辨识基参数，$\boldsymbol{\epsilon}$ 为加性高斯白噪声。

### 3.1 D-optimal 的推导

#### A. 几何推导：超椭球体积最小化

在 $p$ 维参数空间中，置信超椭球的几何体积 $\text{Vol}$ 公式为：

$$\text{Vol} = \frac{\pi^{p/2}}{\Gamma\left(\frac{p}{2} + 1\right)} \cdot \frac{c^p}{\sqrt{\det(\mathbf{F})}}$$

由于 $\det(\mathbf{F}) = \prod_{i=1}^p \lambda_i(\mathbf{F})$（特征值之积），要使误差椭球的**体积最小**，等价于**最大化 $\det(\mathbf{F})$**（即 D-最优准则）。

#### B. 信息论推导：香农微分熵最小化

已知测量数据 $\boldsymbol{\tau}$ 后，待估参数 $\boldsymbol{\theta}$ 后验分布的香农微分熵（代表不确定性总量）为：

$$H(\boldsymbol{\theta} \mid \boldsymbol{\tau}) = \frac{1}{2} \ln \left( (2\pi e)^p \det(\text{Cov}(\hat{\boldsymbol{\theta}})) \right) = \text{常数} - \frac{1}{2} \ln \det(\mathbf{F})$$

最大化 $\det(\mathbf{F})$，即等价于**最小化参数后验分布的信息熵**，从而提取最充沛的测量信息量。

### 3.2 条件数的推导：矩阵摄动与噪声放大

根据矩阵摄动理论（Perturbation Theory），对于线性方程组 $\mathbf{W} \boldsymbol{\theta} = \boldsymbol{\tau}$，当输入力矩存在测量噪声 $\delta \boldsymbol{\tau}$ 时，导致参数估计偏差 $\delta \boldsymbol{\theta}$ 的上界满足：

$$\frac{\Vert{}\delta \boldsymbol{\theta}\Vert{}}{\Vert{}\boldsymbol{\theta}\Vert{}} \le \kappa(\mathbf{W}) \cdot \frac{\Vert{}\delta \boldsymbol{\tau}\Vert{}}{\Vert{}\boldsymbol{\tau}\Vert{}}$$

其中 $\kappa(\mathbf{W})$ 为回归矩阵 $\mathbf{W}$ 的条件数：

$$\kappa(\mathbf{W}) = \frac{\sigma_{\max}(\mathbf{W})}{\sigma_{\min}(\mathbf{W})} = \sqrt{\frac{\lambda_{\max}(\mathbf{F})}{\lambda_{\min}(\mathbf{F})}}$$

**物理含义**：条件数代表**测量噪声转化为参数估计误差的最大放大倍数**。若 $\kappa(\mathbf{W}) = 100$，意味着传感器中 1% 的测量噪声，在最坏的方向上会导致 100% 的参数估计偏差。

---

## 4. 深层对比与核心差异

### 4.1 特征值乘积 vs. 特征值比值

设费雪信息阵 $\mathbf{F}$ 的特征值为 $\lambda_1, \lambda_2, \dots, \lambda_p$：

* **D-optimal** $\propto \prod_{i=1}^p \lambda_i$
* **条件数** $\kappa = \sqrt{\lambda_{\max} / \lambda_{\min}}$

| 场景 | 特征值情况 | D-optimal 表现 | 条件数 $\kappa$ 表现 | 真实的物理辨识效果 |
| --- | --- | --- | --- | --- |
| **场景 A** | $\lambda_1 = 100, \lambda_2 = 100$ | $\det = 10000$（极优） | $\kappa = 1$（极优） | **完美**：各方向均有剧烈运动激励，信噪比极高。 |
| **场景 B** | $\lambda_1 = 0.01, \lambda_2 = 0.01$ | $\det = 0.0001$（极差） | $\kappa = 1$（极优） | **灾难**：机器人几乎未动，信号被噪声淹没；但因特征值同等微弱，条件数呈现虚假优秀。 |
| **场景 C** | $\lambda_1 = 1000, \lambda_2 = 10$ | $\det = 10000$（极优） | $\kappa = 10$（一般） | **可用**：总体激励极强，但参数 1 的激励强度高于参数 2。 |

> **关键推论**：不能单独依赖条件数。条件数仅衡量“对称度”，无法衡量“激励总能量”。

### 4.2 坐标变换与量纲不变性（Scale Invariance）

* **D-optimal（物理尺度不敏感）**：
若对参数进行线性缩放（如改变单位 $\boldsymbol{\theta}' = \mathbf{T} \boldsymbol{\theta}$），新的回归矩阵变为 $\mathbf{W}' = \mathbf{W}\mathbf{T}^{-1}$。
优化目标满足：
$$\arg\max \det(\mathbf{W}'^T \mathbf{W}') = \arg\max \left( \det(\mathbf{T}^{-1})^2 \cdot \det(\mathbf{W}^T \mathbf{W}) \right)$$


由于 $\det(\mathbf{T}^{-1})^2$ 为常数，**D-optimal 算出的最优轨迹解完全独立于参数单位的选取**。
* **条件数（物理尺度极其敏感）**：
转动惯量单位若由 $\text{kg}\cdot\text{m}^2$ 更改为 $\text{g}\cdot\text{cm}^2$，数值放大 $10^7$ 倍，$\mathbf{W}$ 的对应列会被缩小，导致 $\kappa(\mathbf{W})$ 剧烈变化。**因此计算条件数前必须做归一化处理**。

### 4.3 非线性求解器友好度

* **D-optimal（全空间光滑连续）**：
转化为对数形式后为 $-\ln\det(\mathbf{F}) = -2\sum \ln(L_{ii})$。该目标函数在正定流形上是 **$C^\infty$ 无限可导且凸性良好** 的，能直接与 IPOPT 等基于梯度的 NLP 求解器契合。
* **条件数（特征值交叉点不可导）**：
条件数依赖于奇异值极值比。在优化迭代中，当最大/最小奇异值对应的特征向量方向发生交换时（**Eigenvalue Crossing**），条件数关于轨迹参数的梯度会出现非连续跳变，易导致求解器震荡或无法收敛。

---

## 5. 工程落地与工业界标准解法

为了兼顾“强能量激励”与“各向同性抗噪”，工业界通常采用 **D-optimal 作为主目标，配合条件数做约束限制** 的组合策略。

### 5.1 列标准化（Column Normalization）

在代入计算前，首先消除质量（kg）与转动惯量（$\text{kg}\cdot\text{m}^2$）等物理量纲影响，将回归矩阵的每一列缩放至 2-范数为 1：

$$\tilde{\mathbf{W}} = \mathbf{W} \cdot \mathbf{D}^{-1}, \quad \mathbf{D} = \text{diag}(\Vert{}\mathbf{w}_1\Vert{}_2, \Vert{}\mathbf{w}_2\Vert{}_2, \dots, \Vert{}\mathbf{w}_p\Vert{}_2)$$

### 5.2 混合约束优化数学模型

求解轨迹参数 $\boldsymbol{p}$（如有限傅里叶级数系数）的标准非线性规划（NLP）表达如下：

$$\begin{aligned} \min_{\boldsymbol{p}} \quad & -2 \sum_{i=1}^p \ln \Big( \text{diag}\big( \text{chol}(\tilde{\mathbf{W}}(\boldsymbol{p})^T \tilde{\mathbf{W}}(\boldsymbol{p})) \Big)_i \Big) \\ \text{s.t.} \quad & \kappa(\tilde{\mathbf{W}}(\boldsymbol{p})) \le \kappa_{\max} \quad (\text{工程经验常设为 } 10 \sim 30) \\ & \boldsymbol{q}_{\min} \le \boldsymbol{q}(t) \le \boldsymbol{q}_{\max} \quad (\text{关节位置限制}) \\ & \vert{}\dot{\boldsymbol{q}}(t)\vert{} \le \dot{\boldsymbol{q}}_{\max} \quad (\text{关节速度限制}) \\ & \vert{}\ddot{\boldsymbol{q}}(t)\vert{} \le \ddot{\boldsymbol{q}}_{\max} \quad (\text{关节加速度限制}) \\ & \boldsymbol{\tau}_{\min} \le \boldsymbol{\tau}(t) \le \boldsymbol{\tau}_{\max} \quad (\text{电机驱动扭矩限制}) \end{aligned}$$

---

## 6. 总结对比表

| 维度 | D-最优 (D-Optimal) | 条件数 (Condition Number) |
| --- | --- | --- |
| **数学表达式** | $\min -\ln\det(\mathbf{W}^T\mathbf{W})$ | $\min \frac{\sigma_{\max}(\mathbf{W})}{\sigma_{\min}(\mathbf{W})}$ |
| **置信椭球控制** | **控制椭球总体积**（总体不确定性/综合误差） | **控制椭球长短轴比例**（形状偏心率/各向同性） |
| **物理本质** | **最大化总信息量**，自发增强激励强度 | **约束各向同性**，防止单轴方向噪声极化放大 |
| **信噪比 (SNR)** | **高**（自发激励大振幅与加速度运动） | **可能极低**（需额外施加力矩/速度下限约束） |
| **求导与收敛** | **全空间连续光滑可导**，求解器高效 | **存在特征值交叉不可导点**，求解器易震荡 |
| **量纲敏感度** | **不敏感**（自带尺度不变性） | **极度敏感**（必须显式执行列归一化） |
| **工程推荐** | **推荐作为主目标函数** | **推荐作为不等式约束条件** |