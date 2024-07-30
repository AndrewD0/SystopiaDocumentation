# SystopiaDocumentation

## Swing-Up Control

### Background
We employ a physics-based energy approach to implement the swing-up functionality of our inverted pendulum. That is, by determining the energy of the pendulum at different points of a swing, we can send a varying input to the motor that will give the pendulum enough energy to reach the balance point.

To start, we derive the energy of the uncontrolled inverted pendulum
$$E=\frac{1}{2}J\dot{\theta}^2+mgl(\cos\theta-1)$$
which defines the point of 0 energy to be upright position of the pendulum. The input into the motor will be the control law 
$$\mu = \text{sat}_{ng}\left(k(E - E_0) \, \text{sign}(\dot{\theta} \cos\theta)\right)$$
where $\text{sat}_{ng}$ is a linear function that has a max value (saturates) at $ng$, an proportional constant multiplied by the acceleration due to gravity. Furthermore $k$ also represents another proportional constant used for tuning. $E_0$ is the energy of the pendulum in its upright position, 0. We note that the control output will decrease as the pendulum approaches the upright balance point. $\text{sign}(\dot{\theta}\cos\theta)$ determines the direction that the motor will drive the pendulum toward.

### Implementation
The swing-up functionality for the pendulum is done in the $\textbf{control}$ function. To switch from swing-up to PID balancing, we track the angle of the pendulum at every iteration. Once the pendulum is within a certain threshold to the upright position, we then switch from the swing-up function to PID balancing.
