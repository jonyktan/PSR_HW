Note: solution.md best previewed in VS code (ctrl+shift+v)

# 1.1 Navigation without Safe Control

In "flat_evade.py", calculate distance between agents after every time step using `np.linalg.norm`(). Log distance over 500 time steps and calculate average after 500 time steps.

[Video was recorded](Solution\videos\Q1-1.mkv) and average distance was calculated to be 96.98.

# 2.1 Energy function controller (optional)

Given $ \mathbf{c_e} = [x_e, y_e, \dot{x_e}, \dot{y_e}] $, $ \mathbf{c_o} = [x_o, y_o, \dot{x_o}, \dot{y_o}] $ and $ \mathbf{u} = [\ddot{x_e}, \ddot{y_e}] $.

Distance $d$ between ego and obstacle is $ d = \sqrt{(x_e-x_o)^2+(y_e-y_o)^2} \geq 0 $.

Define initial safety index $\phi_0 = d_{min} - d \leq 0 $. 

Then $ \dot{\phi_0} = -\dot{d} $, which does not contain $\mathbf{u}$. 

$ \ddot{\phi_0} = -\ddot{d} $, which contains elements of $\mathbf{u}$: $\ddot{x}_e$ and $\ddot{y}_e$.

So the safety index for control can be $ \phi = c + d_{min}^2 - d^2 - k\dot{d} $ where $c>0, k>0$. Choose power 2 just to avoid the square root in $d$.

Since $ \dot{\phi} $ can be expressed in terms of elements of $x_e$ and $x_o$, partial derivatives of $\phi$ w.r.t $ x_e $ and $ c_o $ do not seem to be needed.

<!-- $ \frac{\partial \phi}{\partial \mathbf{c_e}} = -2\mathbf{c_e} + 2\mathbf{c_o} $ 

$ \frac{\partial \phi}{\partial \mathbf{c_e}} = -2\mathbf{c_o} + 2\mathbf{c_e} $  -->

# 2.2 Safe set controller

Continuing from 2.1, find ${\mathbf{u}}$ by solving the following optimization:

$\min \| \mathbf{u} - \mathbf{u_{ref}} \|,
\\ \text{s.t. }  \dot{\phi} < - \eta \text{ or }  \phi < 0 
\ (\eta \text{ is the safety margin used in } \phi), 
\\ -\mathbf{u}_{max} \leq \mathbf{u} \leq \mathbf{u}_{max} $

## Check feasibility for forward invariance
For feasibility of control, $ \underset{x}{\max} \ \underset{\mathbf{u} \in \Omega }{\min} \ {\dot{\phi} + \eta(\phi)} \leq 0 $.

For forward invariance, i.e. when $ \phi = 0, \eta(\phi) =0 $.

$\dot{\phi} + \eta(\phi) = -2d\dot{d} - k\ddot{d} + 0 $

$ \underset{\mathbf{u} \in \Omega }{\min} \ {\dot{\phi} + \eta(\phi)} \leq 0  \Rightarrow -2d\dot{d} - k\underset{\mathbf{u}}{\max}\ \ddot{d} \leq 0, \forall \mathbf{c}_e, \text{s.t. }\phi(\mathbf{c}_e)=0 $.

Case 1: $ d^2 \leq c+ d_{min}^2  \Rightarrow \dot{d} \geq 0 $ (from equation of $\phi = 0$).

Hence $ -2d\dot{d} - k\underset{\mathbf{u}}{\max}\ \ddot{d} \leq 0 $.

Case 2: $ d^2 > c+ d_{min}^2 $, then 

$\dot{d} = \frac{(x_e-x_o)(\dot{x_e}-\dot{x_o})+(y_e-y_o)(\dot{y_e}-\dot{y_o})\,}{\sqrt{(x_e-x_o)^2+(y_e-y_o)^2}}\,  \\ = \frac{x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel}}{\sqrt{x_{rel}^2+y_{rel}^2}} < 0 $.

Hence $ -2d\dot{d} - k\underset{\mathbf{u}}{\max}\ \ddot{d} 
\\= -2(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel} )\, - k\underset{\mathbf{u}}{\max}\ [\,\frac{\dot{x}_{rel}^2+x_{rel}\ddot{x}_{rel}+\dot{y}_{rel}^2+y_{rel}\ddot{y}_{rel}}{\sqrt{x_{rel}^2+y_{rel}^2}} -\frac{(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel})\,^2}{(\,x_{rel}^2+y_{rel}^2)\,^{\frac{3}{2}}} ]\, 
\\ = -2(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel} )\, -k \frac{\dot{x}_{rel}^2+\dot{y}_{rel}^2}{\sqrt{x_{rel}^2+y_{rel}^2}} +k\frac{(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel})\,^2}{(\,x_{rel}^2+y_{rel}^2)\,^{\frac{3}{2}}} - k\underset{\ddot{x}_e,\ddot{y}_e}{\max}\ \frac{x_{rel}\ddot{x}_e+y_{rel}\ddot{y}_e}{\sqrt{x_{rel}^2+y_{rel}^2}} $ 

 if we take acceleration of the human to be $0$. <i><<What does this show?>></i>

$ c = 1000, k = 5.0, d_{min} = 15.0, \eta = 1$ given in `flat_evade_agent_1.yaml`

## Goal

From CVXOPT documentation, find equation of the form: 

$ \underset{\mathbf{u}}{\min} \frac{1}{2} \mathbf{u}^\mathsf{T}P\mathbf{u} + q^\mathsf{T}\mathbf{u} \\ \text{s.t.} G\mathbf{u} \leq h $

\
$ \underset{\mathbf{u}}{\min} \| \mathbf{u} - \mathbf{u_{ref}} \|
\\ \Rightarrow \underset{\ddot{x}_e, \ddot{y}_e}{\min} [\,(\,\ddot{x}_e-\ddot{x}_{ref})\,^2 + (\,\ddot{y}_e-\ddot{y}_{ref})\,^2]\,
\\ = \underset{\ddot{x}_e, \ddot{y}_e}{\min} [\,\ddot{x}_e^2 - 2\ddot{x}_e\ddot{x}_{ref} + \ddot{x}_{ref}^2 + \ddot{y}_e^2 - 2\ddot{y}_e\ddot{y}_{ref} + \ddot{y}_{ref}^2]\, 
\\ = \underset{\ddot{x}_e, \ddot{y}_e}{\min} [\,\begin{bmatrix} \ddot{x}_e & \ddot{y}_e \end{bmatrix} \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} \ddot{x}_e \\ \ddot{y}_e \end{bmatrix} + \begin{bmatrix} -2\ddot{x}_{ref} & -2\ddot{y}_{ref} \end{bmatrix} \begin{bmatrix} \ddot{x}_e \\ \ddot{y}_e \end{bmatrix} + \ddot{x}_{ref}^2 + \ddot{y}_{ref}^2]\, 
\\ \Rightarrow \underset{\mathbf{u}}{\min} [\,\frac{1}{2} \mathbf{u}^\mathsf{T} \begin{bmatrix} 2 & 0 \\ 0 & 2 \end{bmatrix} \mathbf{u} + \begin{bmatrix} -2\ddot{x}_{ref} \\ -2\ddot{y}_{ref} \end{bmatrix} ^\mathsf{T} \mathbf{u} ]\, $


\
$ \dot{\phi} < - \eta
\\ \Rightarrow -2d\dot{d} - k\ddot{d} < - \eta
\\ \Rightarrow -2(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel} )\, -k \frac{\dot{x}_{rel}^2+\dot{y}_{rel}^2}{\sqrt{x_{rel}^2+y_{rel}^2}} +k\frac{(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel})\,^2}{(\,x_{rel}^2+y_{rel}^2)\,^{\frac{3}{2}}} - k\frac{x_{rel}\ddot{x}_e+y_{rel}\ddot{y}_e}{\sqrt{x_{rel}^2+y_{rel}^2}} < - \eta
\\ \Rightarrow - k\frac{x_{rel}\ddot{x}_e+y_{rel}\ddot{y}_e}{\sqrt{x_{rel}^2+y_{rel}^2}} < - \eta + 2(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel} )\, +k \frac{\dot{x}_{rel}^2+\dot{y}_{rel}^2}{\sqrt{x_{rel}^2+y_{rel}^2}} - k\frac{(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel})\,^2}{(\,x_{rel}^2+y_{rel}^2)\,^{\frac{3}{2}}} 
\\ \Rightarrow -\frac{k}{\sqrt{x_{rel}^2+y_{rel}^2}} \begin{bmatrix} x_{rel} & y_{rel} \end{bmatrix} \mathbf{u}  < - \eta + 2(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel} )\, +k \frac{\dot{x}_{rel}^2+\dot{y}_{rel}^2}{\sqrt{x_{rel}^2+y_{rel}^2}} - k\frac{(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel})\,^2}{(\,x_{rel}^2+y_{rel}^2)\,^{\frac{3}{2}}} $ 

\
So QP standard form is:

$ \min [\,\frac{1}{2} \mathbf{u}^\mathsf{T} \begin{bmatrix} 2 & 0 \\ 0 & 2 \end{bmatrix} \mathbf{u} + \begin{bmatrix} -2\ddot{x}_{ref} \\ -2\ddot{y}_{ref} \end{bmatrix} ^\mathsf{T} \mathbf{u} ]\,
\\ \text{s.t.} \begin{bmatrix} -\frac{k}{\sqrt{x_{rel}^2+y_{rel}^2}} x_{rel} & -\frac{k}{\sqrt{x_{rel}^2+y_{rel}^2}} y_{rel} \\ 1 & 0 \\ 0 & 1 \\ -1 & 0 \\ 0 & -1 \end{bmatrix} 
\mathbf{u}  < 
\begin{bmatrix}- \eta + 2(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel} )\, +k \frac{\dot{x}_{rel}^2+\dot{y}_{rel}^2}{\sqrt{x_{rel}^2+y_{rel}^2}} - k\frac{(\,x_{rel}\dot{x}_{rel}+y_{rel}\dot{y}_{rel})\,^2}{(\,x_{rel}^2+y_{rel}^2)\,^{\frac{3}{2}}} \\ u_{max} \\ u_{max} \\ u_{max} \\ u_{max} \end{bmatrix}
$

<br>

# 2.3 Navigation with Safe Control

## Observations
1. cvxopt solver could not solve for $\mathbf{u}$ when the constraints for $\mathbf{u}_i \leq u_{max}$ were included.
    ```
    File ".../lib/python3.8/site-packages/cvxopt/misc.py", line 450, in update_scaling
        s[:m] = base.sqrt( s[:m] )
    ValueError: domain error
    ````
1. By removing $\mathbf{u}_i \leq u_{max}$ constraint, cvxopt solver could find $\mathbf{u}$:
    - the robot avoids the human (distance at which avoidance behaviour is triggered, is affected by parameter $ c $, hence reduce $c$ value so robot has more opportunity to approach the goal) ([seen in video](Solution\videos\Q2-3-2.mp4))
    - robot safety score is always 0 (no collisions)
    - to get higher score, increase $ k $
    - increase $ \eta $ to increase responsiveness of robot to human.
