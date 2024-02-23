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
,
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
,
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
### Q6
Poles of $G(s)$ are $[2.98, -0.428 &plusmn 3.366i, -3.84]$.

Zeros of $G_1(s)$ are $&plusmn 18.7$ and the zero of $G_2(s)$ is 0.

_Coefficients less than 1e-4 are considered to be 0._
### Q7
**Non-linear system**

The non-linear system is simulated as below.
<img src="/readme_images/non_lin_sys.png">
The simulation results of the non-linear system with a PID controller is shown below.
| 3D render | Step response|
| --- | --- |
| <img src="/readme_images/non_lin_pid.gif"> | <img src="/readme_images/non_lin_res.png"> |

**Linearized system**

The linearized system is also tuned using a PID controller, with the following properties.

<img src="/readme_images/lin_sys.jpg">

The step response is shown below.
<img src="/readme_images/lin_res.png">
This system cannot be stabilized using a compensator.
### Q8
If we remove the unstable pole of the system, the linearized system will be stabilized, but the main non-linear system will still remain unstable.

**Non-linear system**
<img src="/readme_images/non_lin_pole_sys.png">
The simulation results of the non-linear system are presented as follows.
| 3D render | Step response|
| --- | --- |
| <img src="/readme_images/non_lin_pole_pid.gif"> | <img src="/readme_images/non_lin_pole_res.png"> |

**Linearized system**

The linearized system is also tuned using a PID controller, with the following properties.

<img src="/readme_images/lin_pole_sys.png">

The simulation results of the linearized system are presented as follows.
| States | Outputs|
| --- | --- |
| <img src="/readme_images/lin_pole_states.png" width="500" height="350"> | <img src="/readme_images/line_pole_out.png" width="535" height="262"> |

## Phase 2
### Q1
Two state feedbacks are designed with fast and slow poles.
* Fast poles = $[-3, -4, -5, -6]$
* Slow poles = $[-0.5, -0.7, -0.9, -1.1]$

| \ | Fast poles | Slow poles |
| --- | --- | --- |
| State feedback gain | $K_{fast}=[20.0993, 13.9591, 34.2198, 4.9373]$ | $K_{slow}=[5.1497, 0.0771, 0.9613, 0.4268]$ |
| Step response | <img src="https://github.com/fardinabbasi/Ball_and_Beam_Control_System/blob/main/readme_images/fast_step_res.jpg"> | <img src="https://github.com/fardinabbasi/Ball_and_Beam_Control_System/blob/main/readme_images/slow_step_res.jpg"> |
| Outputs | <img src="https://github.com/fardinabbasi/Ball_and_Beam_Control_System/blob/main/readme_images/fast_out.png"> | <img src="https://github.com/fardinabbasi/Ball_and_Beam_Control_System/blob/main/readme_images/slow_out.png"> 

As evident from both systems, the slower poles exhibit smoother movement and require more time to converge. However, a challenge arises when adjusting the poles: the final output value changes accordingly, unless the system converges to zero.

Therefore, if the system is slow, the final value will be large, whereas a design with fast poles will yield a smaller final value, as demonstrated by this example, approximately 0.07.
### Q2
A white noise with a power of 0.3 and a 10-second sampling time is introduced into the system.

The outputs for both state feedbacks are depicted below.
| \ | Fast poles | Slow poles |
| --- | --- | --- |
| Outputs | <img src="https://github.com/fardinabbasi/Ball_and_Beam_Control_System/blob/main/readme_images/d_fast_out.png"> | <img src="https://github.com/fardinabbasi/Ball_and_Beam_Control_System/blob/main/readme_images/d_slow_out.png"> |

The final value changes every time the disturbance is added.
### Q3
A reference tracker is designed with state feedback and integral control with the following properties.

$$
\left(\begin{array}{cc} 
\dot{x}\\
\dot{q}
\end{array}\right) = \left(\begin{array}{cc} 
A & 0\\
-C & 0
\end{array}\right)
\left(\begin{array}{cc} 
x\\
q
\end{array}\right) + 
\left(\begin{array}{cc} 
B\\
0
\end{array}\right) u +
\left(\begin{array}{cc} 
0\\
I
\end{array}\right)r
$$

$$
\overline{A} = \left(\begin{array}{cc} 
0 & 1 & 0 & 0 & 0\\
-0.378 & 0 & 7.0147 & 0.0343 & 0\\
0 & 0 & 0 & 1 & 0\\
18.9 & 0 & -0.3797 & -1.713&0\\
-1 & 0 & 0 & 0 & 0
\end{array}\right)
, 
\overline{B} = \left(\begin{array}{cc} 
0\\
-0.0699\\
0\\
3.4965\\
0
\end{array}\right)
$$

Since $(\overline{A}, \overline{B})$ is controllable, a state feedback gain is designed with poles at $[-1, -3, -4 , -5,-6]$.
* State feedback gain: $K=[34.05 , 18.85 , 39.64 , 5.32 , -14.69]$.

| States | Outputs |
| --- | --- |
| <img src="https://github.com/fardinabbasi/Ball_and_Beam_Control_System/blob/main/readme_images/ref_state.png"> | <img src="https://github.com/fardinabbasi/Ball_and_Beam_Control_System/blob/main/readme_images/ref_out.png"> |
### Q4
A white noise with a power of 0.3 and a 20-second sampling time is introduced into the system.

| States | Outputs |
| --- | --- |
| <img src="https://github.com/fardinabbasi/Ball_and_Beam_Control_System/blob/main/readme_images/d_ref_state.png"> | <img src="https://github.com/fardinabbasi/Ball_and_Beam_Control_System/blob/main/readme_images/d_ref_out.png"> |

As shown, whenever a disturbance is added, the system attempts to track the input and eliminate the disturbance.
### Q5
Since $(A, C)$ is observable, a full-order Luenberger observer is designed with poles at $[-4,-6,-8,-10]$.

$\dot{\hat{X}}=A\hat{X}+Bu+L(y-\hat{y})$

_where_

$$
L = \left(\begin{array}{cc} 
15.9&0.88\\
58.95&13.4\\
0.468&10.38\\
21.22&14.5\\
\end{array}\right)
$$

The step response of the linearized system with the aforementioned state feedback $K_{fast}$ from the observer is depicted below.

<img src="https://github.com/fardinabbasi/Ball_and_Beam_Control_System/blob/main/readme_images/o_fast_out.png">

### Q6
A reduced-dimensional state estimator is desined based on the following procedure.

* $\dot{X} = A_{n\times n}X_{n\times 1} + B_{p\times n}u_{p\times 1}$
* $y_{l\times 1} = C_{l\times n}X_{n\times 1}$

1. Select an arbitrary $n \times n$ stable matrix $F$ that has no eigenvalues in common with those of $A$.

$$
F = \left(\begin{array}{cc} 
-6 & 0\\
0 & -8\\
\end{array}\right)
$$

2. Select an arbitrary $n \times 1$ vector $l$ such that $(F,l)$ is controllable.

$$
l = \left(\begin{array}{cc} 
1 & 0\\
0 & 1\\
\end{array}\right)
$$

3. Solve the unique $T$ in the Lyapunov equation $TA-FT=lC$. Note that $T$ is an $(n-1)\times n$ matrix.

$$
T = \left(\begin{array}{cc} 
0.1908 & -0.0318 & 0.0367 & -0.0083\\
0.0482 & -0.006 & 0.1293 & -0.0205
\end{array}\right)
$$

4. Then the $(n-1)$ dimensional state equation

$$ 
\dot{z} = Fz+TBu+ly
$$

$$
\hat{X} = \left(\begin{array}{cc} 
C\\
T\\
\end{array}\right)^{-1}
\left(\begin{array}{cc} 
y\\
z\\
\end{array}\right)
$$

is an estimate of $X$.

The step response of the linearized system with the aforementioned state feedback $K_{fast}$ from the reduced-dimensional observer is depicted below.
| States | Error |
| --- | --- |
| <img src="/readme_images/rd_state.png"> | <img src="/readme_images/rd_error.png"> |

_where_ $error = z-TX$.
### Q7
The step response of the non-linear system with the aforementioned state feedback $K_{fast}$ from the reduced-dimensional observer is depicted below.
| States | Error |
| --- | --- |
| <img src="/readme_images/non_lin_rd_state.png"> | <img src="/readme_images/non_lin_rd_error.png"> |

_where_ $error = z-TX$.
