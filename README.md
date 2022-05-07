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
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\tilde{z}_k=z_k-\hat{z}_k=h_k(x_k,u_k,v_k)-h_k(\hat{x}_k^-,u_k,\bar{v}_k)}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\hat{z}_k=h_k(x_k,u_k,v_k)&plus;\hat{C}_k(x_k-\hat{x}_k^-)&plus;\hat{D}_k(v_k-\bar{v}_k)}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\hat{C}_k=\frac{dh_k(x_k,u_k,v_k)}{dx_k}|_{x_k=\hat{x}_k^-},\&space;\hat{D}_k=\frac{dh_k(x_k,u_k,v_k)}{dv_k}|_{v_k=\bar{v}_k}}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}L_k=\Sigma_{\tilde{x}\tilde{z},k}^-\Sigma_{\tilde{z},k}^{-1}\simeq\Sigma_{\tilde{x},k}^-\hat{C}_k^T[\hat{C}_k\Sigma_{\tilde{x},k}^-\hat{C_k}^T&plus;\hat{D}_k\Sigma_{\tilde{v}}\hat{D}_k^T]^{-1}}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\hat{x}_k^&plus;=\hat{x}_k^-&plus;L_k(z_k-\hat{z}_k)}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\Sigma_{\tilde{x},k}^&plus;=\Sigma_{\tilde{x},k}^--L_k\Sigma_{\tilde{z},k}L_k^T}">

## 2&3. Central Difference Kalman Filter(CDKF) and Unscented Kalman Filter (UKF)
Given mean and covariance, create 2 more input points that are located gamma standard deviation away from the mean.

<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\mathcal{X}=\{\bar{x},\bar{x}&plus;\gamma\sqrt{\Sigma_{\bar{x}}},\bar{x}-\gamma\sqrt{\Sigma_{\bar{x}}}\}}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\bar{x}=\sum_{i=0}^p\alpha_{i}^{(m)}\mathcal{X}_i,\&space;\Sigma_{\bar{x}}=\sum_{i=0}^p\alpha_{i}^{(c)}(\mathcal{X}_i-\bar{x})(\mathcal{X}_i-\bar{x})^T}">
with tuning parameters shown in table below:
<p>
  <img 
    width="500"
    src="images/parameters.PNG"
  >
</p>

## Final Result (Comparing EKF, CDKF, and UKF)
<p align="center">
  <img 
    width="700"
    src="images/nonlinear_KF.png"
  >
</p>

## Reference
[1] Eric A. Wan, Rudolph van der Merwe. The Unscented Kalman Filter for Nonlinear Estimation. Oregon Graduate Institute of Science & Technology, Feb 2000

Code Progress: ![100%](https://progress-bar.dev/100)

Readme Progress: ![33%](https://progress-bar.dev/50)
