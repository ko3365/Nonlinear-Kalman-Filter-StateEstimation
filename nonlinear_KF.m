clc;clear;close all;
%% Nonlinear KF Initialization
% Variable Init
Sig_w = 2.5; %Process Noise
Sig_v = 1; %Sensor Noise

%Initial State
x = 2+randn(1); xhat = 2; u = 0;
SigX = 1;

%Variable Store
iteration = 50;
xstore = zeros(1,iteration+1);
xstore(1,1) = x; % initial value
xhatstore = zeros(1,iteration);
SigXstore = zeros(1^2,iteration);

%nonlinear System  x = (4+x)^(1/3)+w+2*u,  x^3+3*v

%% Extended KF
for k=1:iteration

    %true value simulation
    u_prev = u;
    u = sin(k/pi)/3;
    w = chol(Sig_w)'*randn(1);
    v = chol(Sig_v)*randn(1);
    z = x^3+3*v;
    x = (4+x)^(1/3)+w+2*u;

    %KF Prediction
    Aty = (1/3)*(4+x)^(-2/3);
    Bty = 1;
    xhat = (4+xhat)^(1/3)+2*u_prev;
    SigX = Aty*SigX*Aty'+Bty*Sig_w*Bty';

    Cty = 3*xhat^2;
    Dty = 3;
    zhat = xhat^3;

    %KF Correction
    L = SigX*Cty'/(Cty*SigX*Cty'+Dty*Sig_v*Dty');
    xhat = xhat+L*(z-zhat);
    xhat = max(-4,xhat);
    SigX = SigX-L*Cty*SigX;

    xstore(1,k+1) = x(:);
    xhatstore(1,k) = xhat;
    SigXstore(1,k) = SigX(:);
end


figure(1)
hold on
plot(0:iteration-1,xstore(1:iteration)','k-')
plot(0:iteration-1,xhatstore','r-')
plot(0:iteration-1,xhatstore'+3*sqrt(SigXstore'), 'b--', ...
    0:iteration-1,xhatstore'-3*sqrt(SigXstore'), 'b--')
grid on
legend('true x','estimate x (Extended KF)','error bounds')
title("Extended Kalman Filter")
