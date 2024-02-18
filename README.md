# Ball & Beam Control System
The system to be investigated in this project is a ball and beam system. This system is one of the classical systems in control theory. Due to properties such as nonlinearity and inherent instability caused by gravity and external disturbances such as mechanical sliding, it can be a suitable system for analysis and design.

<img src="/readme_images/ball_beam.jpg">
The ball is allowed to move with one degree of freedom on the beam. One end of the beam is connected to a servo motor via a lever, which determines the height and angle of rotation of the beam by the servo motor. When, for example, the servo motor gear rotates with an angle α, it changes the angle of the beam by θ. By changing the angle of the beam relative to the horizontal position, gravity causes the pendulum to slide along the beam, and as a result, the position of the ball can be moved back and forth by the servo motor and the lever connected to it. The ultimate goal of this system is to control it in such a way that ball remains at a desired point on the beam and external disturbances do not affect its position.

## Systems Modeling
<img src="/readme_images/model.jpg">

To model the losses and friction of the motor and the conveyor belt that transfers force to the beam, we use a damper with a constant of b. And to model the delay that occurs, we use a spring with stiffness K. Ultimately, considering the described conditions, the equations governing the system will be as follows:

$(\frac{J_b}{r^2}+m)\ddot{x} + \frac{1}{r}(mr^2 + J_b)\ddot{\alpha} - mx\dot{\alpha}^2 = mgsin(\alpha)$

$(mx^2 + J_b + J_w)\ddot{\alpha} + (2m\dot{x}x + bl^2)\dot{\alpha} + Kl^2\alpha + \frac{1}{r}(mr^2 + J_b)\ddot{x} - mgxcos(\alpha) = u(t)lcos(a)$

The values of the parameters are written as follows:
| Symbol | Parameter | Value |
| --- | --- | --- |
| $m$ | Ball mass | $0.27kg$ |
| $r$ | Ball radius | $0.02m$ |
| $b$ | Beam friction | $1Ns/m$ |
| $K$ | Spring stiffness | $10^{-3}N/m$ |
| $l$ | Force application distance | $0.49m$ |
| $J_w$ | Moment of inertia of the beam | $14.025\times 10^{-2} kg.m^2$ |
| $J_b$ | Moment of inertia of the ball | $4.32\times 10^{-5} kg.m^2$ |
| $g$ | Gravity of earth | $9.81m/s^2$ |

To study the behavior of the system, 4 state variables, 1 control input, and 2 outputs are defined as follows:
* $x_1$: position of the ball
* $x_2$: velocity of the ball
* $x_3$: Angle of the beam
* $x_4$: Angular velocity of the beam
* $u$: Input voltage
* _Outputs_: position of the ball & angle of the beam

State equations are written as follows:

* $\dot{x_1}=x_2$
* $\dot{x_2}=\frac{A_{22}\times C_1-A_{12}\times C_2}{det(A)}$
* $\dot{x_3}=x_4$
* $\dot{x_4}=\frac{-A_{21}\times C_1+A_{11}\times C_2}{det(A)}$

_Where_

$A_{11} = \frac{J_b}{r^2}+m, A_{12}=\frac{m\times r^2+J_b}{r}, A_{21}=A_{12}, A_{22}=m\times x_1^2+J_w+J_B$

$B_{11}=mgsin(x_3), B_{12}=mx_1x_4^2, B_{21}=lcos(x_3)u, B_{22}=mgx_1cos(x_3), B_{23}=-kl^2x_3, B_{24}=-(2mx_1x_2+bl^2)x_4$

$C_1=B_{11}+B_{12}, C_2=B_{21}+B_{22}+B_{23}+B_{24}$

$det(A)=A_{11}\times A_{22}-A_{12}\times A_{21}$

The state values over a 10-second period are illustrated below.
<img src="/readme_images/equations.png">

## Phase 1
### Q2
The set of differential equations is linearized around the $X_{eq} = (0, 0, 0, 0)^T$ in the following manner:


* $\dot{X} = AX+Bu$
* $Y = CX + Du$

_Where_

$$
A = \left(\begin{array}{cc} 
0 & 1 & 0 & 0\\
-0.378 & 0 & 7.0147 & 0.0343\\
0 & 0 & 0 & 1\\
18.9 & 0 & -0.3797 & -1.7133
\end{array}\right)
$$

$$
B = \left(\begin{array}{cc} 
0\\
-0.07\\
0\\
3.5
\end{array}\right)
$$

$$
C = \left(\begin{array}{cc} 
1 & 0 & 0 & 0\\
0 & 0 & 1 & 0
\end{array}\right)
$$

$$
D = \left(\begin{array}{cc} 
0\\
0
\end{array}\right)
$$
### Q3
Eigen values of the linearized system are outlined as:

$$ \lambda_1 = 2.98, \lambda_2=-3.84, \lambda_{3,4}=-0.428&plusmn3.367i $$

Since $λ_1$ is positive, the system is unstable unless this mode is controllable.
### Q4
The controllability matrix is presented as:

$$
Co = \left(\begin{array}{cc} 
B & AB & A^2B & A^3B\\
\end{array}\right) = \left(\begin{array}{cc} 
0&-0.0699&0.1198&24.3479\\
-0.0699&0.1198&24.3479&-41.8051\\
0&3.4965&-5.9903&7.6137\\
3.4965&-5.9903&7.6137&-8.5053
\end{array}\right)
$$

Since the controllability matrix is full-rank, the system is controllable.

Also, the observability matrix is presented as:

$$
Ob = \left(\begin{array}{cc} 
C\\
CA\\
CA^2\\
CA^3
\end{array}\right) = \left(\begin{array}{cc} 
1 & 0 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 0 & 1\\
-0.3780 & 0 & 7.0147 & 0.0343\\
18.9001 & 0 & -0.3797 & -1.7133\\
0.6476 & -0.3780 & -0.0130 & 6.9560\\
-32.3809 & 18.9001 & 0.6506 & 2.5556
\end{array}\right)
$$

Since the observability matrix is full-rank, the system is observable.

The transform function is written as follows:

$$
G(s) = \left(\begin{array}{cc} 
G_1(s)\\
G_2(s)
\end{array}\right) = \left(\begin{array}{cc} 
\frac{-0.07s^2+24.5}{s^4+1.71s^3+0.75s^2-132.43}\\
\frac{3.5s^2}{s^4+1.71s^3+0.75s^2-132.43}
\end{array}\right)
$$

Since order of the transform function is equal to the number of state variables, the system representation is minimal.
<!--
### Q5
$$
e^{At} = \left(\begin{array}{cc} 
0.274e^{3t}+0.21e^{-3.85t}+e^{-0.43t}(0.516 cos⁡(3.37t)+0.0616 sin⁡(3.37t))&0.092e^{3t}-0.054e^{-3.85t}+e^{-0.43t} (-0.037 cos⁡(3.37t)+0.149 sin⁡(3.37t))& 0.21e^{3t}+0.095e^{-3.85t}+e^{-0.43t}(-0.3 cos⁡(3.37t)-0.116 sin⁡(3.37t))&0.04e^{3t}-0.44e^{-3.85t}+0.09e^{-0.43t} sin⁡(3.37t)\\
0.82e^{3t}-0.807e^{-3.85t}+e^{-0.43t} (-0.014 cos⁡(3.37t)-1.764 sin⁡(3.37t))&0.274e^{3t}+0.21e^{-3.85t}+e^{-0.43t} (0.516 cos⁡(3.37t)+0.0616 sin⁡(3.37t))& 0.63e^{3t}-0.37e^{-3.85t}+e^{-0.43t} (-0.26 cos⁡(3.37t)+1.07 sin⁡(3.37t))&0.13e^{3t}+0.17e^{-3.85t}+e^{-0.43t}(-0.3 cos⁡(3.37t)+0.043 sin⁡(3.37t))\\
0.36e^{3t}+0.46e^{-3.85t}+e^{-0.43t} (-0.82 cos⁡(3.37t)+0.1 sin⁡(3.37t))&0.12e^{3t}-0.12e^{-3.85t}-0.25e^{-0.43t} sin⁡(3.37t)& 0.27e^{3t}+0.21e^{-3.85t}+e^{-0.43t} (0.52 cos⁡(3.37t)+0.06 sin⁡(3.37t))&0.06e^{3t}-0.97e^{-3.85t}+e^{-0.43t} (0.037 cos⁡(3.37t)+0.14 sin(3.37t))\\
1.07e^{3t}-1.78e^{-3.85t}+e^{-0.43} (0.7 cos⁡(3.37t)+2.72 sin⁡(3.37t))& 0.36e^{3t}+0.46e^{-3.85t}+e^{-0.43t} (-0.82 cos⁡(3.37t)+0.1 sin⁡(3.37t))& 0.82e^{3t}-0.8e^{-3.85t}+e^{-0.43t} (-0.14 cos⁡(3.37t)-1.76 sin⁡(3.37t))&0.177e^{3t}+0.371e^{-3.85t}+e^{-0.43t} (0.45 cos⁡(3.37t)-0.18 sin⁡(3.37t))
\end{array}\right)
$$
-->
