function P  = PredictCovariance(deltaAngle, ...
    deltaVelocity, ...
    quat, ...
    states,...
    P, ...  % Previous state covariance matrix
    dt) ... % IMU and prediction time step

% Set filter state process noise other than IMU errors, which are already 
% built into the derived covariance predition equations. 
% This process noise determines the rate of estimation of IMU bias errors
%     dAngBiasSigma = dt*5E-6;
%     dVelBiasSigma = single(dt*0.01/60);

%     dAngBiasSigma = single(dt*0.05/3600*pi/180);
%     dVelBiasSigma = single(dt*0.01/60);
    
    dAngBiasSigma = single(dt*0.2/3600*pi/180);
    dVelBiasSigma = single(dt*0.5/60);
    
 processNoise = [0*ones(1,6), 0*ones(1,3),dAngBiasSigma*[1 1 1],dVelBiasSigma*[1 1 1]].^2;

%     gyrNoise = 0.0035;     
%     accNoise = 0.02;

% Specify the estimated errors on the IMU delta angles and delta velocities
% Allow for 0.5 deg/sec of gyro error
% daxNoise = (dt*0.0035)^2;
% dayNoise = (dt*0.0035)^2;
% dazNoise = (dt*0.0035)^2;
% % Allow for 0.5 m/s/s of accelerometer error
% dvxNoise = (dt*0.02)^2;
% dvyNoise = (dt*0.02)^2;
% dvzNoise = (dt*0.02)^2;

daxNoise = (dt*0.5*pi/180)^2;
dayNoise = (dt*0.5*pi/180)^2;
dazNoise = (dt*0.5*pi/180)^2;
% Allow for 0.5 m/s/s of accelerometer error
dvxNoise = (dt*0.5)^2;
dvyNoise = (dt*0.5)^2;
dvzNoise = (dt*0.5)^2;

dvx = deltaVelocity(1);
dvy = deltaVelocity(2);
dvz = deltaVelocity(3);
dax = deltaAngle(1);
day = deltaAngle(2);
daz = deltaAngle(3);

q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

dax_b = states(10);
day_b = states(11);
daz_b = states(12);

dvx_b = states(13);
dvy_b = states(14);
dvz_b = states(15);

% dax_b = 0;
% day_b = 0;
% daz_b = 0;
% 
% dvx_b = 0;
% dvy_b = 0;
% dvz_b = 0;

% Predicted covariance
F = calcF(dax,dax_b,day,day_b,daz,daz_b,dt,dvx,dvx_b,dvy,dvy_b,dvz,dvz_b,q0,q1,q2,q3);
Q = calcQ(daxNoise,dayNoise,dazNoise,dvxNoise,dvyNoise,dvzNoise,q0,q1,q2,q3);
P = F*P*transpose(F) + Q;

% Add the general process noise
for i = 1:15
    P(i,i) = P(i,i) + processNoise(i);
end

% Force symmetry on the covariance matrix to prevent ill-conditioning
% of the matrix which would cause the filter to blow-up
P = 0.5*(P + transpose(P));

% ensure diagonals are positive
for i=1:15
    if P(i,i) < 0
        P(i,i) = 0;
    end   
end
    
end