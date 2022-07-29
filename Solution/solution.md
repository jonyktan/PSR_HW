## 1.1 Navigation without Safe Control

In "flat_evade.py", calculate distance between agents after every time step using np.linalg.norm(). Log distance over 500 time steps and calculate average after 500 time steps.

Video was recorded and average distance was calculated to be 96.98.

## 2.1 Energy function controller (optional)

Define $\phi$ to be the negative of squared distance between ego and obstacle: 

$ \phi = -\left[(c_{e,x} - c_{o,x})^2 + (c_{e,y} - c_{o,y})^2 \right] \\ = -(\mathbf{c_e}-\mathbf{c_o})^2 $

Therefore partial derivatives of $\phi$ are:

$ \frac{\partial \phi}{\partial \mathbf{c_e}} = -2\mathbf{c_e} + 2\mathbf{c_o} $ 

$ \frac{\partial \phi}{\partial \mathbf{c_e}} = -2\mathbf{c_o} + 2\mathbf{c_e} $ 

## 2.2 Safe set controller

