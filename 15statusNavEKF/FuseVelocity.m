function [...
    quat, ... % quaternion state vector after fusion of measurements
    states, ... % state vector after fusion of measurements
    angErr, ... % angle error
    P, ... % state covariance matrix after fusion of corrections
    innovation,... % NED velocity innovations (m/s)
    varInnov] ... % NED velocity innovation variance ((m/s)^2)
    = FuseVelocity( ...
    quat, ... % predicted quaternion states from the INS
    states, ... % predicted states from the INS
    P, ... % predicted covariance
    measVel) % NED velocity measurements (m/s)



% define persistent variables
persistent velFailCount;
if isempty(velFailCount)
    velFailCount = uint32(0);
end
persistent posFailCount;
if isempty(posFailCount)
    posFailCount = uint32(0);
end
persistent hgtFailCount;
if isempty(hgtFailCount)
    hgtFailCount = uint32(0);
end

gpsRetryTimeNoTAS = single(5);
dt = single(0.02);
maxVelFailCount = round(gpsRetryTimeNoTAS/dt);
maxPosFailCount = maxVelFailCount;
maxHgtFailCount = maxVelFailCount;


velErr = single(0.15*0);
posErr = single(0.15*0);
VelNE_noise = 0.2^2 + velErr^2;
VelD_noise = 0.2^2 + velErr^2;
Pos_noise = 2^2 + posErr^2;
Alt_noise = 2^2 + posErr^2;
    
H_gps = [zeros(6,4),eye(6,6),zeros(6,6)];
%        H_gps = [zeros(3,4),eye(3,3),zeros(3,9)];
R_gps = diag([VelNE_noise;VelNE_noise;VelD_noise;Pos_noise;Pos_noise;Alt_noise]);

innovation = zeros(1,6);
varInnov = zeros(1,6);
% Fuse measurements sequentially
angErrVec = [0;0;0];

observation = zeros(6,1);
    
observation(1) = measVel(1);
observation(2) = measVel(2);
observation(3) = measVel(3);
observation(4) = measVel(4);
observation(5) = measVel(5);
observation(6) = measVel(6);    

for obsIndex = 1:6
    stateIndex = 3 + obsIndex;
    varInnov(obsIndex) = P(stateIndex,stateIndex) + R_gps(obsIndex,obsIndex);
    innovation(obsIndex) = states(stateIndex) - observation(obsIndex);
end

velInnov = innovation(1)^2 + innovation(2)^2 + innovation(3)^2;
if ( velInnov < 25.0*(varInnov(1) + varInnov(2) + varInnov(3)) ||  (velFailCount > maxVelFailCount) )
    velHealth = 1;
    velFailCount = 0;
else
    velHealth = 0;
    velFailCount = velFailCount +1;
end

posInnov = innovation(4)^2 + innovation(5)^2 ;
if( posInnov < 25.0*(varInnov(4) + varInnov(5)) || (posFailCount > maxPosFailCount) )
    posHealth = 1;
    posFailCount = 0;
else
    posHealth = 0;
    posFailCount = posFailCount +1;
end

hgtInnov = innovation(6)^2;
if( hgtInnov < 25.0*varInnov(6) || (hgtFailCount > maxHgtFailCount) )
    hgtHealth = 1;
    hgtFailCount = 0;
else
    hgtHealth = 0;
    hgtFailCount = hgtFailCount + 1;
end
    

for obsIndex = 1:6
     if (velHealth && (obsIndex >= 1 && obsIndex <= 3)) || ...
                    (posHealth && (obsIndex == 4 || obsIndex == 5)) || ...
                    (hgtHealth && (obsIndex == 6))
                stateIndex = 3 + obsIndex;
                % Calculate the velocity measurement innovation
                innovation(obsIndex) = states(stateIndex) - measVel(obsIndex);

                % Calculate the Kalman Gain
                H = zeros(1,15);
                H(1,stateIndex) = 1;
                varInnov(obsIndex) = ( H*P*transpose(H) + R_gps(obsIndex,obsIndex) );
                K = ( P*transpose(H) )/varInnov(obsIndex);

                % Calculate state corrections
                xk = K * innovation(obsIndex);

                % Apply the state corrections
                states(1:3) = 0;
                states = states - xk;

                % Store tilt error estimate for external monitoring
                angErrVec = angErrVec + states(1:3);

                % the first 3 states represent the angular misalignment vector. This is
                % is used to correct the estimated quaternion
                % Convert the error rotation vector to its equivalent quaternion
                % truth = estimate + error
                rotationMag = sqrt(states(1)^2 + states(2)^2 + states(3)^2);
                if rotationMag > 1e-12
                    deltaQuat = [cos(0.5*rotationMag); [states(1);states(2);states(3)]/rotationMag*sin(0.5*rotationMag)];
                    % Update the quaternion states by rotating from the previous attitude through
                    % the error quaternion
                    quat = QuatMult(quat,deltaQuat);
                    % re-normalise the quaternion
                    quatMag = sqrt(quat(1)^2 + quat(2)^2 + quat(3)^2 + quat(4)^2);
                    quat = quat / quatMag;
                end

                % Update the covariance
                P = P - K*H*P;

                % Force symmetry on the covariance matrix to prevent ill-conditioning
                P = 0.5*(P + transpose(P));

                % ensure diagonals are positive
                for i=1:15
                    if P(i,i) < 0
                        P(i,i) = 0;
                    end
                end                
     end
end

angErr = sqrt(dot(angErrVec,angErrVec));

end