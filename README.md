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

#### Preparation:
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\hat{x}^{a,&plus;}_{k-1}=\begin{bmatrix}\hat{x}^&plus;_{k-1}\\\bar{w}\\\bar{v}\end{bmatrix},\Sigma^{a,&plus;}_{\bar{x},k-1}=\text{diag}(\Sigma_{\tilde{x},k-1}^{&plus;},\Sigma_{\tilde{w}},\Sigma_{\tilde{v}})}&space;">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\mathcal{X}^{a,&plus;}_{k-1}=\{\hat{x}^{a,&plus;}_{k-1},\hat{x}^{a,&plus;}_{k-1}&plus;\gamma\sqrt{\Sigma^{a,&plus;}_{\bar{x},k-1}},\hat{x}^{a,&plus;}_{k-1}-\gamma\sqrt{\Sigma^{a,&plus;}_{\bar{x},k-1}}\}}">

#### Prediction:
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\mathcal{X}^{x,-}_{k,i}=f_{k-1}(\mathcal{X}^{x,&plus;}_{k-1,i},u_{k-1},\mathcal{X}^{w,&plus;}_{k-1,i}),\hat{x}_k^-\simeq\sum^p_{i=0}\alpha_i^{(m)}\mathcal{X}^{x,-}_{k,i}}&space;">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\Sigma^-_{\tilde{x},k}=\sum^p_{i=0}\alpha_i^{(c)}(\mathcal{X}^{x,-}_{k,i}-\hat{x}^-_k)(\mathcal{X}^{x,-}_{k,i}-\hat{x}^-_k)^T}&space;">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\mathcal{Z}_{k,i}=h_k(\mathcal{X}^{x,-}_{k,i},u_k,\mathcal{X}^{v}_{k,i}),&space;\hat{z}=\sum_{i=0}^p\alpha_i^{(m)}\mathcal{Z}_{k,i}}&space;">

#### Correction:
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\Sigma_{\tilde{z},k}=\sum_{i=0}^p{\alpha_i^{(c)}(\mathcal{Z}_{k,i}-\hat{z}_k)(\mathcal{Z}_{k,i}-\hat{z}_k)^T}}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\Sigma_{\tilde{x}\tilde{z},k}^-=\sum_{i=0}^p{\alpha_i^{(c)}(\mathcal{X}_{k,i}^{x,-}-\hat{x}_k^-)(\mathcal{Z}_{k,i}-\hat{z}_k)^T}}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}L_k=\Sigma_{\tilde{x}\tilde{z},k}^-\Sigma_{\tilde{z},k}^{-1}}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\hat{x}_k^&plus;=\hat{x}_k^-&plus;L_k(z_k-\hat{z}_k)}">
<img src="https://latex.codecogs.com/svg.image?\large&space;{\color{Gray}\Sigma_{\tilde{x},k}^&plus;=\Sigma_{\tilde{x},k}^--L_k\Sigma_{\tilde{z},k}L_k^T}">


## Final Result (Comparing EKF, CDKF, and UKF)
<p align="center">
  <img 
    width="700"
    src="images/nonlinear_KF.png"
  >
</p>

## Reference
[1] Eric A. Wan, Rudolph van der Merwe. The Unscented Kalman Filter for Nonlinear Estimation. Oregon Graduate Institute of Science & Technology, Feb 2000

