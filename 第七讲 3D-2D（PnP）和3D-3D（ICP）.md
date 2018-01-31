# 第七讲 3D-2D（PnP）和3D-3D（ICP）


标签（空格分隔）： SLAM

---

PnP Perspective-n-Point 已知n个3D空间点及其投影位置，求解相机运动。

```flow
st=>start: start
cond1=>condition: 单目？
cond2=>condition: 双目？
cond3=>condition: RGBD?
op1=>operation: 初始化：对极约束+三角化
op2=>operation: 三角化
op3=>operation: RGBD深度图
op4=>operation: 得到世界坐标（3D）和归一化相机坐标（2D）
op5=>operation: PnP
end=>end: 相机位姿R，t

st->cond1
cond1(yes)->op1->op4
cond1(no)->cond2
cond2(yes)->op2->op4
cond2(no)->cond3
cond3(yes)->op3->op4->op5->end
```

---
**DLT 直接线性变换**
需要6对匹配点

- 已知：
 - 空间点P的齐次**世界坐标**$P=(X,Y,Z,1)^T$
 - 特征点的**归一化相机坐标**$x_1=(u_1,v_1,1)^T$

 $s$$
\left(
\begin{matrix}
u_1\\v_1\\1
\end{matrix}
\right)
$$=
\left(
\begin{matrix}
t_1,t_2,t_3,t_4\\
t_5,t_6,t_7,t_8\\
t_9,t_{10},t_{11},t_{12}
\end{matrix}
\right)$$
\left(
\begin{matrix}
X\\Y\\Z\\1
\end{matrix}
\right)
$

- T矩阵即为摄像机矩阵$[R|t]$
- 六个这样的P点（即六对3D-2D匹配）即可解出T矩阵
- 由于未考虑R的内在约束，所以要针对DLT估计的T左边的3*3矩阵块，寻找一个最好的R与它进行近似，方法是QR分解（相当于把结果投影到SE(3)流形上，转换成旋转和平移两部分）

---
**P3P**
需要3对匹配

- 已知：
 - A，B，C三个点的世界坐标(3D）
 - a，b，c三个点的归一化相机坐标(2D）
 
- 求解出投影点a，b，c的相机坐标(3D)
- 使问题简化为3D-3D位姿求解（相机坐标和世界坐标），用ICP求解

---
**SLAM中，通常先使用P3P/EPnP估计相机位姿，在构建最小二乘优化问题对估计值进行调整(BA)**

---
**BA (Bundle Adjusyment)**

- PnP等方法只是对位姿进行了优化，BA是把位姿和观测（点的世界坐标）都作为变量同时进行优化（这就是捆集调整的含义）
- 用最小二乘法（GN，LM）最小化重投影误差
 重投影误差出现的原因是对相机位姿的估计不够准确，BA使得对R，t的估计更接近真实值


- **重投影误差**=真实的像素坐标-恢复的像素坐标
 - 真实的像素坐标为观测的投影位置
 - 恢复的像素f坐标为从估计出的世界坐标重投影到图像上的像素坐标

![此处输入图片的描述][1]
1. 重投影误差
$s_i$$
\left[
\begin{matrix}
u_i\\v_i\\1
\end{matrix}
\right]
$$=Kexp(\xi$^$)
\left[
\begin{matrix}
X_i\\Y_i\\Z_i\\1
\end{matrix}
\right]$


         $s_iu_i=Kexp(\xi$^$)P_i$
         
最小二乘误差函数: $\xi^*=argmin{1\over2}\sum{||u_i-{1\over{s_i}}Kexp(\xi$^$)P_i||^2_2}$

- 线性化：$e(x+\Delta x)=e(x)+J\Delta x$

---
**3D-3D :  ICP**
迭代最近点

- 线性求解：SVD
- 非线性优化：BA


  [1]: http://images2015.cnblogs.com/blog/457232/201706/457232-20170612151415587-1801983887.png