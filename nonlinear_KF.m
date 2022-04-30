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

% Input, Output store for comparison: 
ustore = zeros(1,iteration+1);
ustore(1,1) = u;
zstore = zeros(1,iteration);

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
    
    % Store Data for Plotting
    xstore(1,k+1) = x(:);
    xhatstore(1,k) = xhat;
    SigXstore(1,k) = SigX(:);
    
    % Later Use for Comparison
    ustore(1,k+1)=u;
    zstore(1,k) = z;
end

%% Central Difference Kalman Filter
Nxa = 3;
h = sqrt(3);
alpham(1) = (h*h-Nxa)/(h*h);
alpham(2) = 1/(2*h*h);
alphamk = alpham(2);
alphac = alpham;
alpha = [alpham(1) alphamk(:,ones([1 2*Nxa]))]';


xhat2 = 2;
SigX2 = 1;

xhatstore2 = zeros(iteration,length(xhat2));
SigXstore2 = zeros(iteration,length(xhat2)^2);

for k=1:iteration
    
    % Prediction - Obtain Sigma Points
    xhat_aug = [xhat2;0;0];
    Sig_aug = blkdiag(SigX2,Sig_w,Sig_v);
    sqr_Sig_aug=chol(Sig_aug,'lower');
    X = xhat_aug(:,ones([1 2*Nxa+1]))+h*[zeros(Nxa,1), sqr_Sig_aug,-sqr_Sig_aug];
    % Prediction - xhat, zhat
    X_x = (4+X(1,:)).^(1/3)+X(2,:)+2*ustore(1,k);
    xhat2 = X_x*alpha;
    square_X = (X_x(:,2:end) - xhat2(:,ones(1,2*Nxa)))*sqrt(alphac(2));
    square_X1 =X_x(:,1)-xhat2;
    SigX2 = square_X*square_X'+alphac(1)*square_X1*square_X1;
    Z = X_x.^3+3*X(3,:);
    zhat2 = Z*alpha;

    % Correction
    square_Z = (Z(:,2:end)-zhat2*ones(1,2*Nxa))*sqrt(alphac(2));
    square_Z1 = Z(:,1)-zhat2;
    SigZ = square_Z*square_Z'+alphac(1)*square_Z1*square_Z1';
    SigXZ = square_X*square_Z'+alphac(1)*square_X1*square_Z1';
    Lsp = SigXZ/SigZ;

    xhat2 = xhat2+Lsp*(zstore(1,k)-zhat2);
    SigX2 = SigX2-Lsp*SigZ*Lsp';

    xhatstore2(k,:) = xhat2;
    SigXstore2(k,:) = SigX2(:);
end

%% Unscented Kalman Filter
nxa = 3;
alp = 0.2;
kepa = 3-nxa;
beta = 2;
lambda = alp^2*(nxa+kepa)-nxa;
alpham1 = lambda/(nxa+lambda);
alphamk = 1/(2*(nxa+lambda));
alphac1 = lambda/(nxa+lambda)+(1-alp^2+beta);
alphack = 1/(2*(nxa+lambda));
alpha_mean = [alpham1 alphamk(:,ones(1,2*Nxa))]';
alpha_cov = [alphac1 alphack(:,ones(1,2*nxa))]';
xhat3 = 2;
SigX3 = 1;

xhatstore3 = zeros(iteration,length(xhat3));
SigXstore3 = zeros(iteration,length(xhat3)^2);

for k=1:iteration
    
    % Prediction - Obtain Sigma Points
    xhat_aug = [xhat3;0;0];
    Sig_aug = blkdiag(SigX3,Sig_w,Sig_v);
    sqr_Sig_aug=chol(Sig_aug,'lower');
    X = xhat_aug(:,ones([1 2*nxa+1]))+sqrt(nxa+lambda)*[zeros(nxa,1), sqr_Sig_aug,-sqr_Sig_aug];
    % Prediction - xhat, zhat
    X_x = (4+X(1,:)).^(1/3)+X(2,:)+2*ustore(1,k);
    xhat3 = X_x*alpha_mean;
    square_X = (X_x(:,2:end) - xhat3(:,ones(1,2*nxa)))*sqrt(alphack);
    square_X1 =X_x(:,1)-xhat3;
    SigX3 = square_X*square_X'+alphac1*square_X1*square_X1;
    Z = X_x.^3+3*X(3,:);
    zhat3 = Z*alpha_mean;

    % Correction
    square_Z = (Z(:,2:end)-zhat3*ones(1,2*nxa))*sqrt(alphack);
    square_Z1 = Z(:,1)-zhat3;
    SigZ = square_Z*square_Z'+alphac1*square_Z1*square_Z1';
    SigXZ = square_X*square_Z'+alphac1*square_X1*square_Z1';
    L_unscent = SigXZ/SigZ;

    xhat3 = xhat3+L_unscent*(zstore(1,k)-zhat3);
    SigX3 = SigX3-L_unscent*SigZ*L_unscent';

    xhatstore3(k,:) = xhat3;
    SigXstore3(k,:) = SigX3(:);
end

%% Plot
figure(1)

subplot(3,1,1)
hold on
plot(0:iteration-1,xstore(1:iteration)','k-')
plot(0:iteration-1,xhatstore','r-')
plot(0:iteration-1,xhatstore'+3*sqrt(SigXstore'), 'b--', ...
    0:iteration-1,xhatstore'-3*sqrt(SigXstore'), 'b--')
grid on
legend('true x','estimate x (Extended KF)','error bounds')
title("Extended Kalman Filter")
ylim([-2,13])
hold off

subplot(3,1,2)
hold on
plot(0:iteration-1,xstore(1:iteration)','k-')
plot(0:iteration-1,xhatstore2','r-')
plot(0:iteration-1,xhatstore2'+3*sqrt(SigXstore2'), 'b--', ...
    0:iteration-1,xhatstore2'-3*sqrt(SigXstore2'), 'b--')
grid on
legend('true x','estimate x (CDKF)','error bounds')
ylim([-2,13])
title("Central Difference Kalman Filter")
hold off

subplot(3,1,3)
hold on
plot(0:iteration-1,xstore(1:iteration)','k-')
plot(0:iteration-1,xhatstore3','r-')
plot(0:iteration-1,xhatstore3'+3*sqrt(SigXstore3'), 'b--', ...
    0:iteration-1,xhatstore3'-3*sqrt(SigXstore3'), 'b--')
grid on
legend('true x','estimate x (UKF)','error bounds')
ylim([-2,13])
title("Unscented Kalman Filter")
hold off
