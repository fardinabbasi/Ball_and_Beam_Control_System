# Ball & Beam Control System
The system to be investigated in this project is a ball and beam system. This system is one of the classical systems in control theory. Due to properties such as nonlinearity and inherent instability caused by gravity and external disturbances such as mechanical sliding, it can be a suitable system for analysis and design.

<img src="/readme_images/ball_beam.jpg">
The ball is allowed to move with one degree of freedom on the beam. One end of the beam is connected to a servo motor via a lever, which determines the height and angle of rotation of the beam by the servo motor. When, for example, the servo motor gear rotates with an angle α, it changes the angle of the beam by θ. By changing the angle of the beam relative to the horizontal position, gravity causes the pendulum to slide along the beam, and as a result, the position of the ball can be moved back and forth by the servo motor and the lever connected to it. The ultimate goal of this system is to control it in such a way that ball remains at a desired point on the beam and external disturbances do not affect its position.

## Systems Modeling
<img src="/readme_images/model.jpg">

To model the losses and friction of the motor and the conveyor belt that transfers force to the beam, we use a damper with a constant of b. And to model the delay that occurs, we use a spring with stiffness K. Ultimately, considering the described conditions, the equations governing the system will be as follows:

$(J_b/r^2+m)\ddot{x} + 1/r(mr^2 + J_b)\ddot{\alpha} - mx\dot{\alpha}^2 = mgsin(\alpha)$

$(mx^2 + J_b + J_w)\ddot{\alpha} + (2m\dot{x}x + bl^2)\dot{\alpha} + Kl^2\alpha + 1/r(mr^2 + J_b)\ddot{x} - mgxcos(\alpha) = u(t)lcos(a)$

| Symbol | Parameter | Value |
| --- | --- | --- |
| $m$ | ball mass | $0.27kg$ |
| $r$ | ball radius | $0.02m$ |
| $b$ | beam friction | $1Ns/m$ |
| $K$ | spring stiffness | $10^{-3}N/m$ |
| $l$ | force application distance | $0.49m$ |
| $J_w$ | Moment of inertia of the beam | $14.025\times 10^{-2} kg.m^2$ |
| $J_b$ | Moment of inertia of the ball | $4.32\times 10^{-5} kg.m^2$ |
| $g$ | Gravity of earth | $9.81m/s^2$ |
