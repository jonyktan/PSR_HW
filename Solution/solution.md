## 1.1 Navigation without Safe Control

In "flat_evade.py", calculate distance between agents after every time step using np.linalg.norm(). Log distance over 500 time steps and calculate average after 500 time steps.

Video was recorded and average distance was calculated to be 96.98.

## 2.1 Energy function controller (optional)

Given $ \mathbf{c_e} = [x_e, y_e, \dot{x_e}, \dot{y_e}] $, $ \mathbf{c_o} = [x_o, y_o, \dot{x_o}, \dot{y_o}] $ and $ \mathbf{u} = [\ddot{x_e}, \ddot{y_e}] $.

Distance $d$ between ego and obstacle is $ d = \sqrt{(x_e-x_o)^2+(y_e-y_o)^2} \geq 0 $.

Define initial safety index $\phi_0 = d_{min}^2 - d^2 \leq 0 $ , $d_{min} = 15.0$ given in `flat_evade_agent_1.yaml`. Choose power 2 just to avoid the square root in $d$.

Then $ \dot{\phi_0} = -2d\dot{d} \\ = -2[\,(x_e-x_o)(\dot{x_e}-\dot{x_o})+(y_e-y_o)(\dot{y_e}-\dot{y_o})\,]\,  \\ = -2[x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel} ] \ $, which does not contain $\mathbf{u}$. 

$ \ddot{\phi_0} = -2\dot{d}^2-2d\ddot{d} \\ = -2[ x_{rel}(\ddot{x_e}-\ddot{x_o}) + \dot{x}_{rel}^2 + y_{rel}(\ddot{y_e}-\ddot{y_o}) + \dot{y}_{rel}^2 ] \ \\ = -2[ x_{rel}\ddot{x_e} + \dot{x}_{rel}^2 + y_{rel}\ddot{y_e} + \dot{y}_{rel}^2 ] $ if we take acceleration of the human to be $0$.

So the safety index for control can be $ \phi = c + \phi_0 + k\dot{\phi_0} $ where $c>0, k>0$.

$ \phi = c + d_{min}^2 - d^2 - 2kd\dot{d} $

Partial derivatives of $\phi$ are not needed.

<!-- $ \frac{\partial \phi}{\partial \mathbf{c_e}} = -2\mathbf{c_e} + 2\mathbf{c_o} $ 

$ \frac{\partial \phi}{\partial \mathbf{c_e}} = -2\mathbf{c_o} + 2\mathbf{c_e} $  -->

## 2.2 Safe set controller

Continuing from 2.1, find ${\mathbf{u}}$ by solving the following optimization:

$\min || \mathbf{u} - \mathbf{u_{ref}} ||$,

$ \text{s.t. }  \dot{\phi} < - \eta \text{ or }  \phi < 0 $ ($\eta$ is the safety margin used in $\phi$)

For feasibility of control, $ \underset{x}{\max} \ \underset{\mathbf{u} \in \Omega }{\min} \ {\dot{\phi} + \eta(\phi)} \leq 0 $.

For forward invariance, i.e. when $ \phi = 0, \eta(\phi) =0 $.

$\dot{\phi} + \eta(\phi) = \dot{\phi_0} + k\ddot{\phi_0} \\ = -2d\dot{d} -2k\dot{d}^2-2kd\ddot{d} $

<!-- $\dot{\phi} + \eta(\phi) = -2d\dot{d} -2k\dot{d}^2 - 2kd\ddot{d}  \ $ -->
<!-- $ \dot{\phi_0} + k \ddot{\phi_0} \\ = -\dot{d} - k\|{\mathbf{u}}\| $ -->

$ \underset{\mathbf{u} \in \Omega }{\min} \ {\dot{\phi} + \eta(\phi)} \leq 0  \Rightarrow -2d\dot{d} -2k\dot{d}^2-2kd \ \underset{\mathbf{u}}{\max}\ \ddot{d} \leq 0, \forall \mathbf{c}, \text{s.t. }\phi(\mathbf{c})=0 $.

Case 1: $ d^2 \leq c+ d_{min}^2  \Rightarrow d\dot{d} \geq 0 $ (from equation of $\phi = 0$).

Hence $ -2d\dot{d} -2k\dot{d}^2-2kd \ \underset{\mathbf{u}}{\max}\ \ddot{d} \leq 0 $.

Case 2: $ d > c+ d_{min} $, then $\dot{d} = \sqrt{(\dot{x_e}-\dot{x_o})^2+(\dot{y_e}-\dot{y_o})^2} > 0 $.
