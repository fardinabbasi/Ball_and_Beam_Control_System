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
* 
State equations are written as follows:

* $\dot{x_1}=x_2$
* $\dot{x_2}=\frac{A_{22}\times C_1-A_{12}\times C_2}{det(A)}$
* $\dot{x_3}=x_4$
* $\dot{x_4}=\frac{-A_{21}\times C_1+A_{11}\times C_2}{det(A)}$

Where

$A_{11} = \frac{J_b}{r^2}+m, A_{12}=\frac{m\times r^2+J_b}{r}, A_{21}=A_{12}, A_{22}=m\times x_1^2+J_w+J_B$

$B_{11}=mgsin(x_3), B_{12}=mx_1x_4^2, B_{21}=lcos(x_3)u, B_{22}=mgx_1cos(x_3), B_{23}=-kl^2x_3, B_{24}=-(2mx_1x_2+bl^2)x_4$

$C_1=B_{11}+B_{12}, C_2=B_{21}+B_{22}+B_{23}+B{24}$

$det(A)=A_{11}\times A_{22}-A_{12}\times A_{21}$
