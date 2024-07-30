# SystopiaDocumentation

## Swing-Up Control

### Background
We employ a physics-based energy approach to implement the swing-up functionality of our inverted pendulum. That is, by determining the energy of the pendulum at different points of a swing, we can send a varying input to the motor that will give the pendulum enough energy to reach the balance point. To start, we derive the energy of the uncontrolled inverted pendulum:

$$E=\frac{1}{2}J\dot{\theta}^2+mgl(\cos\theta-1)$$

which defines the point of 0 energy to be upright position of the pendulum. The input into the motor will be the control law 

$$\mu = \text{sat}_{ng}\left(k(E - E_0) \, \text{sign}(\dot{\theta} \cos\theta)\right)$$

where $\text{sat}_{ng}$ is a linear function that has a max value (saturates) at $ng$, an proportional constant multiplied by the acceleration due to gravity. Furthermore $k$ also represents another proportional constant used for tuning. $E_0$ is the energy of the pendulum in its upright position, 0. We note that the control output will decrease as the pendulum approaches the upright balance point. $\text{sign}(\dot{\theta}\cos\theta)$ determines the direction that the motor will drive the pendulum toward.

### Implementation
The swing-up functionality for the pendulum is done in the $\textbf{control}$ function. To switch from swing-up to PID balancing, we track the angle of the pendulum at every iteration. Once the pendulum is within a certain threshold to the upright position, we then switch from the swing-up function to PID balancing. For the proportional constants, we have $n = 90$ and $k = 150$.

### Notes on Implementation
#### Sampling Frequency
We noticed that sampling frequency has an effect on the overall behaviour of the swing-up controller. For example, the frequency of measurements to calculate the angular velocity (which is done through backwards difference) can have an effect on the reliability of the swing-up. Currently, our implementation measures the angular velocity of the pendulum every 6 ms.

#### Angular Encoder Accuracy
We have noticed issues with the angular encoder. Firstly, at times the angular encoder can produce an oscillating measurement even when the pendulum is perfectly still. We have noticed that this affects the reliability of the swing-up controller. Moreover, at times the angular encoder can completely disconnect from power, which then produces a constant 0 measurement. This is due to the wiring of the angular encoder which is connected directly to the STM32. Powering the STM32 using separate wiring would solve this.

