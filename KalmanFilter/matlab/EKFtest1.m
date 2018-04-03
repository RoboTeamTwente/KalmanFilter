% clear all
% close all
%%
% load data
vision = load('visionRelevantData.txt');
vision2 = vision;
xsens = load('xsensRelevantData.txt');
xsens2=xsens;

% crop and extract data
offset=4;
A1 = 1341+offset*50; A2 = 1580+offset*100; B = 300;
xsens = xsens(A2:A2+B*2,:); %why choose from row 1541 to 2141?
    u_xs = -xsens(:,2) - 0.0; %+ 0.0;
    v_xs = xsens(:,1) - 0.0;% + 0.06; % gravity term?
    a_xs = (xsens(:,3))/180*pi; %-xsens2(1,3)
    t_xs = (1:length(xsens))/100;
vision = vision(A1:A1+B,:);
    x_vis = vision(:,1)-vision(1,1);
    y_vis = vision(:,2)-vision(1,2);
    a_vis = vision(:,3)-vision2(1,3);
    t_vis = (1:length(vision))/50;
    
    odd = 1:2:length(xsens);
    a_xs2 = a_xs(odd);

% put data in vectors and generate cam measurements as if sampled at xsens frequency
u = [u_xs,v_xs,a_xs]';
z = ones(2,length(u))*nan;
f_cam = 50;     % cam data frequency
f_xs = 100;     % xsens data frequency
ind = ceil(1:f_xs/f_cam:length(u));

z(:,ind) = [x_vis,y_vis]';

% parameters
var_a = 0.01^2;     % variance of acceleration input from xsens
var_theta = 0.01^2; % variance of orientation input from xsens
var_xpos = 0.005^2;  % variance of position process noise
var_xvel = 0.2^2;  % variance of velocity process noise
var_z = 0.01^2;     % variance of camera position data

delt = 1/f_xs;      % time-step xsens data (s)
delay = 0.08;       % delay of cam data wrt xsens data (s)
N_ahead = round(delay/delt);    % time-steps that cam lags behind xsens
% generate delay in camera data wrt xsens data 
z = [zeros(2,N_ahead),z];

% constant matrices
Cu = diag([var_a, var_a, var_theta]); % input process noise covmat
Cn = eye(2)*var_z;    % measurement noise covmat
Cw = diag([var_xpos, var_xpos, var_xvel, var_xvel]); % state process noise covmat

H = [eye(2),zeros(2)];                      % measurement matrix
F = [eye(2),delt*eye(2);zeros(2),eye(2)];   % system matrix

% initialization
x_pred = [0;0;0;0];     % initial state vector
Cx_pred = diag([1,1,0.1,0.1]);     % initial covmat of state vector
K = length(u);

%% EKF
for k = 1+N_ahead:K
    % Update
        zk = z(:,k);    % current camera measurement
        % If no measurement -> take the prediction
        if ( isnan( zk(1) ) )
            x_upd = x_pred;
            Cx_upd = Cx_pred;
        else % In normal case -> update
            S = H*Cx_pred*H' + Cn;              % innovation matrix
            K = Cx_pred*H'/S;                   % Kalman gain matrix
            x_upd = x_pred + K*(zk-H*x_pred);	% x(i|i)
            Cx_upd = Cx_pred - K*S*K';          % Cx(i|i)
        end;
    
    % Logging
        x_est{k} = x_upd;   % log of estimation x(i|i)
        Cx_est{k} = Cx_upd;	% log of covmat Cx(i|i)
    
    % Prediction
        uk = u(:,k-N_ahead); % take xsens data N steps back, such that it syncs with camera data
        x_pred = F*x_upd + delt*gfunc(uk);    % one step prediction
        G = delt*Gjacobian(uk);               % input jacobian
        Cx_pred = F*Cx_upd*F' + G*Cu*G' + Cw; % covmat of the prediction
        
    	% propagate prediction upto latest available xsens measurement
        x_ahead = x_pred;
        for j=1:N_ahead
            x_ahead = F*x_ahead + delt*gfunc(u(:,k-N_ahead+j));
        end
        x_est_ahead{k} = x_ahead; % log ahead prediction
end

%% PLOTTING
x_est2=cell2mat(x_est);
x_est2 = [ones(4,N_ahead)*nan,x_est2];
x_ahead = [ones(4,N_ahead*2)*nan,cell2mat(x_est_ahead)];

% plot positions
figure(1); clf; hold on
%     plot(z(1,:),z(2,:),'bx');
%     plot(x_est2(1,:),x_est2(2,:),'o');
%     plot(x_ahead(1,:),x_ahead(2,:),'+');
    plot(z(1,:),'bx');
    plot(x_est2(1,:),'o');
    plot(x_ahead(1,:),'+');
    legend('vision x pos','estimated x pos (delayed)','real-time estimated x pos (propagated)');
    
% plot velocities
figure(2); clf; hold on;
    ind = (1:2:length(x_vis)*2-2);
    vx=zeros(1,length(x_vis)*2-2);
    vx(ind) = (x_vis(2:end)-x_vis(1:end-1))*50;
    vx(ind+1) = vx(ind);
    vx = [zeros(1,N_ahead+1),vx];
    plot(vx,'b');
    plot(x_est2(3,:));
    plot(x_ahead(3,:)','-');
    legend('x vel derived from vision','estimated x vel (delayed)','real-time estimated x vel (propagated)');
    
figure(1);
 