# Nonlinear Kalman Filter 
## 1. Extended Kalman Filter
EKF uses Taylor series to linearize the system equation. 

#### Prediction step:

<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\hat{x}_k^-=\mathbb{E}[f_{k-1}(x_{k-1},u_{k-1},w_{k-1})|\mathbb{Z}_{k-1}]\simeq&space;f_{k-1}(\hat{x}_{k-1}^&plus;,u_{k-1},\bar{w}_{k-1})}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\tilde{x}_k^-=x_k-\hat{x}_k^-=f_{k-1}(x_{k-1},u_{k-1},w_{k-1})-f_{k-1}(\hat{x}_{k-1}^&plus;,u_{k-1},\bar{w}_{k-1})}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\hat{x}_k^-\simeq&space;f_{k-1}(x_{k-1},u_{k-1},w_{k-1})&plus;\hat{A}_{k-1}(x_{k-1}-\hat{x}_{k-1}^&plus;)&plus;\hat{B}_{k-1}(w_{k-1}-\bar{w}_{k-1})}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\hat{A}_{k-1}=\frac{df_{k-1}(x_{k-1},u_{k-1},w_{k-1})}{dx_{k-1}}|_{x_{k-1}=\hat{x}_{k-1}^&plus;},\&space;\hat{B}_{k-1}=\frac{df_{k-1}(x_{k-1},u_{k-1},w_{k-1})}{dw_{k-1}}|_{w_{k-1}=\bar{w}_{k-1}^&plus;}}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\Sigma_{\tilde{x},k}^-=\hat{A}_{k-1}\Sigma_{\tilde{x},k-1}^&plus;\hat{A}_{k-1}&plus;\hat{B}_{k-1}\Sigma_{\tilde{w}}\hat{B}_{k-1}}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\hat{z}_k=\mathbb{E}[h_k(x_k,u_k,v_k|\mathbb{Z}_{k-1})]\simeq&space;h_k(\hat{x}_k^-,u_k,\bar{v}_k)}">

#### Correction Step:





Code Progress: ![100%](https://progress-bar.dev/100)

Readme Progress: ![20%](https://progress-bar.dev/0)
