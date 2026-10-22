# 第 3 章 MINCO 算法原理

本章目标：从优化问题出发理解 MINCO 到底在算什么，读懂 GCOPTER 中 `MINCO_S3NU` 的每一行，理解本项目的时间分配策略，并在 MATLAB 中亲手验证这些性质。

参考论文：Z. Wang, X. Zhou, C. Xu, F. Gao, *Geometrically Constrained Trajectory Optimization for Multicopters*, IEEE Transactions on Robotics, 2022。

## 3.0 准备：在 MATLAB 中直接调用 MINCO

本章的实验需要离线调用 MINCO，先编译 MEX（只需一次）：

```matlab
startup_platform

% 生成 build/minco_mex.*，与 Simulink 使用同一份 C 代码
```

用法：

```matlab
[ver, T, Ttot] = minco_mex('plan', slot, dim, W, vMax, aMax, timeScale);  % W: dim × 路径点数
[p, v, a, j, seg] = minco_mex('eval', slot, ver, t);                       % t 可以是向量
```

## 3.1 问题：穿过路径点的"最平滑"轨迹

给定：

- 路径点 \(q_0, q_1, \dots, q_N\)（共 \(N+1\) 个，\(N\) 段）
- 每段时长 \(T_1, \dots, T_N\)
- 起点和终点的状态：位置、速度、加速度（本项目中速度、加速度都为 0）

求一条轨迹 \(p(t)\)，使它在第 \(i\) 个时刻精确经过 \(q_i\)，并且**最小化 jerk（加加速度）的能量**：

\[
\min_{p(t)} \; J = \int_0^{T_\Sigma} \left\| p^{(3)}(t) \right\|^2 dt,
\qquad T_\Sigma = \sum_i T_i
\]

为什么是 jerk？

| 最小化的量 | 名称 | 典型应用 |
|---|---|---|
| \(\|p^{(2)}\|^2\) 加速度 | minimum acceleration（s=2） | 简单移动 |
| \(\|p^{(3)}\|^2\) jerk | **minimum jerk（s=3）** | **机械臂**：jerk 小意味着电机力矩变化平缓、冲击和振动小 |
| \(\|p^{(4)}\|^2\) snap | minimum snap（s=4） | 四旋翼：推力与 snap 直接相关 |

一般地，记最小化 \(s\) 阶导数为 "s 阶问题"。GCOPTER 提供了 `MINCO_S2NU`、`MINCO_S3NU`、`MINCO_S4NU` 三个类，后缀 `NU` 表示 non-uniform time（每段时长可以不同）。本项目用 `MINCO_S3NU`。

## 3.2 最优解的结构

用变分法（Euler-Lagrange 方程）可以证明，这个问题的最优解有非常简洁的结构：

1. **每段都是 \(2s-1\) 次多项式**。s=3 时是 5 次多项式，每段每维 6 个系数：
   \[
   p_i(t) = c_{i,0} + c_{i,1} t + c_{i,2} t^2 + c_{i,3} t^3 + c_{i,4} t^4 + c_{i,5} t^5,\quad t \in [0, T_i]
   \]
2. **在中间路径点处 \(2s-2\) 阶连续**。s=3 时位置、速度、加速度、jerk、snap 都连续（C4）。
3. **解唯一**，而且由路径点和时长**唯一确定**。

第 2 点很关键：它告诉我们最优轨迹除了"经过路径点"之外，还**自动满足**多少阶连续条件。于是整个优化问题被化成了一组线性方程。

> 这就是 MINCO（MINimum COntrol）的核心思想：**最小控制量轨迹可以由路径点 \(q\) 和时间 \(T\) 直接参数化**，\((q, T) \mapsto c\) 是一个可以快速计算、且可微的映射。后续的轨迹优化（避障、动力学约束）只需要在 \((q, T)\) 这个小得多的空间里搜索。

## 3.3 线性方程组：数一数条件

每维有 \(6N\) 个未知系数。条件数：

| 位置 | 条件 | 个数 |
|---|---|---|
| 起点 | 位置、速度、加速度 | 3 |
| 每个中间点（共 \(N-1\) 个） | 经过路径点 1 个 + 位置/速度/加速度/jerk/snap 连续 5 个 | 6 |
| 终点 | 位置、速度、加速度 | 3 |
| **合计** | | \(3 + 6(N-1) + 3 = 6N\) ✓ |

方程数正好等于未知数，写成：

\[
M(T)\, c = b(q)
\]

- \(M\) 是 \(6N \times 6N\) 矩阵，只和时长 \(T\) 有关
- \(b\) 是右端项，只和路径点、边界状态有关
- **各维共用同一个 \(M\)**，只是 \(b\) 的列不同，所以多维问题就是多个右端项

## 3.4 逐行对照 GCOPTER 源码

打开 `third_party/GCOPTER/gcopter/include/gcopter/minco.hpp`，找到 `MINCO_S3NU::setParameters`。记第 \(i\) 段系数存放在 `b` 的第 `6i` 到 `6i+5` 行（升幂）。

**起点条件**（第 0~2 行）：

```cpp
A(0, 0) = 1.0;     // c_{0,0}         = p(0)
A(1, 1) = 1.0;     // c_{0,1}         = v(0)
A(2, 2) = 2.0;     // 2 c_{0,2}       = a(0)
b.row(0) = headPVA.col(0).transpose();   // 起点位置
b.row(1) = headPVA.col(1).transpose();   // 起点速度
b.row(2) = headPVA.col(2).transpose();   // 起点加速度
```

**第 \(i\) 个中间点**（第 `6i+3` 到 `6i+8` 行），对照多项式求导：

| 行 | 源码中的系数 | 含义 |
|---|---|---|
| `6i+3` | `6, 24T, 60T², ..., -6` | jerk 连续：\(p_i'''(T_i) = p_{i+1}'''(0)\) |
| `6i+4` | `24, 120T, ..., -24` | snap 连续：\(p_i''''(T_i) = p_{i+1}''''(0)\) |
| `6i+5` | `1, T, T², T³, T⁴, T⁵`，右端 `inPs.col(i)` | 经过路径点：\(p_i(T_i) = q_{i+1}\) |
| `6i+6` | `1, T, ..., T⁵, -1` | 位置连续：\(p_i(T_i) = p_{i+1}(0)\) |
| `6i+7` | `1, 2T, 3T², 4T³, 5T⁴, -1` | 速度连续 |
| `6i+8` | `2, 6T, 12T², 20T³, -2` | 加速度连续 |

以 snap 连续为例：\(p''''(t) = 24 c_4 + 120 c_5 t\)，所以第 \(i\) 段末端的 snap 为 \(24 c_{i,4} + 120 T_i c_{i,5}\)，第 \(i+1\) 段起点的 snap 为 \(24 c_{i+1,4}\)，两者相减等于 0，正好对应源码：

```cpp
A(6 * i + 4, 6 * i + 4) = 24.0;
A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
A(6 * i + 4, 6 * i + 10) = -24.0;       // 6(i+1)+4，下一段的 c_4
```

**终点条件**（最后 3 行）：第 \(N\) 段末端的位置、速度、加速度等于 `tailPVA`。

> **思考**：为什么 jerk/snap 连续的两行排在"经过路径点"之前？
> 因为 `BandedSystem::factorizeLU` **不做主元交换**（注释写着 `NO PIVOT`）。行的排列必须保证消元过程中对角元素不为 0，同时让矩阵保持窄带宽。这种细节就是工程实现与教科书公式的差别。

### 带状矩阵与 O(N) 复杂度

每个方程只涉及相邻两段的系数，所以 \(M\) 的非零元素集中在对角线附近（上下带宽都是 6）。`BandedSystem` 只存储这条带：

```cpp
A.create(6 * N, 6, 6);   // 尺寸 6N，下带宽 6，上带宽 6
...
A.factorizeLU();          // 带状 LU 分解，O(N)
A.solve(b);               // 前代 + 回代，结果直接写回 b
```

一般稠密矩阵求解是 \(O(N^3)\)，带状矩阵只需 \(O(N)\)。这就是 MINCO 能在嵌入式上实时运行的原因：几十段轨迹的求解只需要微秒级时间。

## 3.5 本项目的包装：`minco_c_api.cpp`

GCOPTER 的类固定为 3 维（`Eigen::Matrix3d`、`MatrixX3d`），而机械臂需要 6 维。由 3.3 节可知**各维共用同一个 \(M\)、相互独立**，所以直接按 3 维一组分批求解：

```cpp
// GCOPTER 的 MINCO 固定为 3 维；各维相互解耦，因此按 3 维一组分批求解与整体求解等价
for (int g = 0; g < in.dim; g += 3)
{
    ...
    minco::MINCO_S3NU solver;
    solver.setConditions(head, tail, N);
    solver.setParameters(inner, ts);
    const Eigen::MatrixX3d &b = solver.getCoeffs();
    ...
}
```

这样**完全不用修改第三方源码**，升级 GCOPTER 时也不会冲突。

系数读出后存成 `c[段][维][次数]`（升幂），采样时用 Horner 法则计算：

```cpp
p = c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))));
v = c[1] + t * (2.0 * c[2] + t * (3.0 * c[3] + t * (4.0 * c[4] + t * 5.0 * c[5])));
```

Horner 法则只需要 5 次乘法和 5 次加法，不用计算 \(t^k\)，数值上也更稳定。

### 动手实验 3-1：验证最优解的性质

```matlab
W = deg2rad(platformCfg.joint.waypointsDeg).';          % 6 × 5
[ver, T, Ttot] = minco_mex('plan', 2, 6, W, P.vMax, P.aMax, 1.0);
tk = [0 cumsum(T)];                                     % 每个路径点的时刻

% (1) 精确经过路径点
p = minco_mex('eval', 2, ver, tk);
max(abs(p - W), [], "all")                              % ~1e-14

% (2) 中间点处 jerk 连续：左右两侧各取极小偏移
e = 1e-7;
[~, ~, ~, jl] = minco_mex('eval', 2, ver, tk(2:end-1) - e);
[~, ~, ~, jr] = minco_mex('eval', 2, ver, tk(2:end-1) + e);
max(abs(jl - jr), [], "all")                            % ~1e-6，只是 snap × 2e-7 的量级

% (3) 画出位置和 jerk
t = linspace(0, Ttot, 2000);
[p, v, a, j] = minco_mex('eval', 2, ver, t);
tiledlayout(2,1); nexttile; plot(t, p); xline(tk, ":"); title("位置")
nexttile; plot(t, j); xline(tk, ":"); title("jerk（分段二次、处处连续）")
```

## 3.6 时间分配：MINCO 不会告诉你的那部分

MINCO 解决的是"**给定时间**，求最优轨迹"。时间 \(T_i\) 本身怎么定？这是一个独立的问题，常见做法有三种：

| 方法 | 思路 | 本项目 |
|---|---|---|
| 启发式 | 按距离和速度估算 | ✓ 用作初值 |
| 时间缩放 | 整体拉伸到刚好满足限幅 | ✓ |
| 时间优化 | 把 \(T\) 当作优化变量，最小化 \(J + \rho\, T_\Sigma\) + 约束惩罚 | GCOPTER 的完整做法，见第 8 章扩展 |

### 第一步：梯形速度启发式

对每段、每维，假设按"加速 → 匀速 → 减速"的梯形速度曲线运动，所需时间为：

\[
t(d) =
\begin{cases}
\dfrac{d}{v_{\max}} + \dfrac{v_{\max}}{a_{\max}}, & d \ge v_{\max}^2 / a_{\max} \quad\text{（能加速到最大速度）}\\[2mm]
2\sqrt{d / a_{\max}}, & \text{否则（三角形速度曲线）}
\end{cases}
\]

每段时长取所有维度中最慢的那个，即 \(T_i = \max_d t(|q_{i+1,d} - q_{i,d}|)\)，并且不小于 0.05 s。

```cpp
double trapezoidTime(double dist, double v, double a)
{
    dist = std::fabs(dist);
    if (dist * a >= v * v)
    {
        return dist / v + v / a;
    }
    return 2.0 * std::sqrt(dist / a);
}
```

启发式只是估计，用它算出的 MINCO 轨迹，峰值速度、加速度可能超过限幅，也可能远低于限幅。

### 第二步：整体时间缩放

**关键性质（时间缩放等变性）**：如果起点和终点的速度、加速度都为 0，把所有时长乘以 \(k\)，新的最优轨迹就是原轨迹在时间轴上的拉伸：

\[
p_k(t) = p(t / k)
\]

证明思路：

1. 拉伸后的轨迹仍然经过所有路径点，起止速度、加速度乘以 \(1/k\)、\(1/k^2\) 后仍为 0，所以它是可行解
2. 对任意可行轨迹，拉伸后的 jerk 能量都变为原来的 \(k^{-5}\) 倍，是一个正的常数倍
3. 所以拉伸是可行集之间的一一对应，且保持能量的大小顺序，最优解拉伸后仍是最优解

于是速度变为原来的 \(1/k\)，加速度变为 \(1/k^2\)。要让峰值恰好等于限幅，需要的缩放倍数为：

\[
k = \max_d \max\left( \frac{\max_t |v_d(t)|}{v_{\max,d}},\; \sqrt{\frac{\max_t |a_d(t)|}{a_{\max,d}}} \right)
\]

加速度项取平方根，正是因为加速度按 \(1/k^2\) 缩放。

```cpp
// 起止速度/加速度为零时，最小 jerk 解对整体时间缩放是等变的，一次缩放即可贴合限幅
const double k = requiredTimeScale(s, in);
if (k > 1e-9)
{
    for (int i = 0; i < N; i++)
    {
        T[i] = std::max(T[i] * k * in.timeScale, kMinPieceTime);
    }
    ...
}
```

`requiredTimeScale` 在每段上均匀采样 65 个点估计峰值。`k` 可能小于 1（轨迹被压缩、变快），也可能大于 1。最后再乘以用户给的 `timeScale`（≥ 1）留出裕量。

> 注意本项目对**每个关节分别**限幅（盒约束）。GCOPTER 自带的 `getMaxVelRate` 计算的是速度向量的模长，适合四旋翼，但不适合机械臂——各关节的电机能力是独立的。

### 动手实验 3-2：验证时间缩放等变性

```matlab
[v1, T1, Tt1] = minco_mex('plan', 2, 6, W, P.vMax, P.aMax, 1.0);
t = linspace(0, Tt1, 50);
p1 = minco_mex('eval', 2, v1, t);

[v2, T2, Tt2] = minco_mex('plan', 2, 6, W, P.vMax, P.aMax, 2.0);
p2 = minco_mex('eval', 2, v2, 2 * t);    % 在 2 倍时刻采样

Tt2 / Tt1                                 % 2
max(abs(p1 - p2), [], "all")              % 0，同一条几何路径
```

再验证缩放后的峰值恰好贴合限幅：

```matlab
t = linspace(0, Tt1, 20001);
[~, v, a] = minco_mex('eval', 2, v1, t);
max(abs(v) ./ P.vMax, [], "all")          % ≤ 1
max(abs(a) ./ P.aMax, [], "all")          % ≈ 1.000，加速度是这组路径点的瓶颈
```

### 动手实验 3-3：与 MATLAB 内置函数交叉验证

Robotics System Toolbox 中的 `minjerkpolytraj` 求解的是同一个问题。用相同的时刻对比：

```matlab
[ver, T] = minco_mex('plan', 2, 6, W, P.vMax, P.aMax, 1.0);
tk = [0 cumsum(T)];
n = size(W, 2);
BC = nan(6, n);  BC(:, [1 n]) = 0;        % 只约束起终点，中间点自由
[qm, qdm, qddm, ~, ~, ~, tS] = minjerkpolytraj(W, tk, 501, ...
    VelocityBoundaryCondition=BC, AccelerationBoundaryCondition=BC, ...
    JerkBoundaryCondition=nan(6, n));
[pm, vm, am] = minco_mex('eval', 2, ver, tS);
max(abs(qm - pm), [], "all")              % ~1e-12
```

> **踩坑**：`minjerkpolytraj` 默认把**每个路径点**的速度、加速度都约束为 0，这样机械臂会在每个点停一下。不改边界条件时两者差异可达 0.06 rad。用 NaN 表示"自由"后，两者在数值精度内完全一致。

这个实验说明两件事：我们的移植是正确的；MATLAB 内置函数和 GCOPTER 求解的是同一个数学问题，区别在于 MINCO 的 \(O(N)\) 实现和可微结构更适合嵌入式部署与在线优化。

## 3.7 jerk 能量

GCOPTER 提供了解析能量 `getEnergy`。对 5 次多项式，jerk 为 \(p'''(t) = 6c_3 + 24 c_4 t + 60 c_5 t^2\)，平方积分可以展开成系数的二次型，源码中那串 `36.0, 144.0, 192.0, 240.0, 720.0, 720.0` 就是展开后的常数。

### 动手实验 3-4：时间越长，能量越小

```matlab
s = [1 1.2 1.5 2 3];
E = zeros(size(s));
for i = 1:numel(s)
    [ver, ~, Tt] = minco_mex('plan', 2, 6, W, P.vMax, P.aMax, s(i));
    t = linspace(0, Tt, 100001);
    [~, ~, ~, j] = minco_mex('eval', 2, ver, t);
    E(i) = trapz(t, sum(j.^2, 1));
end
loglog(s, E, "o-"); xlabel("timeScale"); ylabel("jerk 能量")
polyfit(log(s), log(E), 1)                % 斜率 ≈ -5
```

斜率为 -5，正好验证了 3.6 节证明中的 \(k^{-5}\)。这也说明"平滑"和"快"是一对矛盾：GCOPTER 的时间优化就是在 \(J + \rho\, T_\Sigma\) 中权衡二者。

## 3.8 本章小结

- MINCO 求解"给定路径点和时间，最小化 s 阶导数能量"的问题，s=3 即最小 jerk
- 最优解是分段 5 次多项式、中间点 C4 连续，归结为一个 \(6N \times 6N\) 带状线性方程组
- 各维共用同一个系数矩阵，6 维可以拆成两组 3 维求解
- 带状 LU 让求解复杂度为 \(O(N)\)，适合实时
- 时间分配：梯形启发式给初值，再利用时间缩放等变性一次缩放贴合限幅
- MINCO 与 `minjerkpolytraj` 在数值精度内一致

## 3.9 练习

1. 在实验 3-1 的图中，jerk 在每段内是什么形状？snap 呢？这与多项式次数有什么关系？
2. 构造一组路径点，使 3-2 中的瓶颈从加速度变成速度（提示：拉大相邻路径点间距）。
3. 阅读 `MINCO_S4NU`，数一数它的 `setConditions` 需要哪些边界状态，每段有几个系数，带宽是多少。
4. 如果起点速度不为 0（例如从运动中重新规划），时间缩放等变性还成立吗？为什么？这对在线重规划意味着什么？

下一章：[第 4 章 C/C++ 代码部署到 Simulink](04_C代码部署到Simulink.md)
