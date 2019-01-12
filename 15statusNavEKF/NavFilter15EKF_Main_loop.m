data_length = size(IMUtime,1);
magIndexlimit = length(mag);

angErr = 0;
headingAligned = 0;
measDec = 0;
time_est(1:data_length) = 0;

quat = [1;0;0;0];
Tbn = Quat2Tbn(quat);

step = (data_length)/100;
hwait=waitbar(0,'NavFilter waitting');

statesInitialised  = 0;

global IMUmsec;

A1 =[];
VelNED_ref = [];
PosNED_ref = [];

quat = [1;0;0;0];
states = single(zeros(15,1));     
P = single(zeros(15,15));   

startDelayTime = 100;
dt = 0.02;

Sigma_angErr = 1; % 1 Sigma uncertainty in angular misalignment (rad)
Sigma_velNED = 0.5; % 1 sigma uncertainty in horizontal velocity components
Sigma_pos_NE = single(15); % 1 sigma uncertainty in horizontal position components
Sigma_pos_D  = single(15); % 1 sigma uncertainty in vertical position
Sigma_dAngBias  = 5*pi/180*dt; % 1 Sigma uncertainty in delta angle bias
Sigma_dVel   = single(0.1*0.02); % 1 Sigma uncertainty in delta velocity bias

covariance   = single( diag([Sigma_angErr*[1;1;1];Sigma_velNED*[1;1;1];Sigma_pos_NE*[1;1];Sigma_pos_D;Sigma_dAngBias*[1;1;1];Sigma_dVel*[1;1;1]].^2 ) );

startIndex = max(11,ceil(startDelayTime/dt));
magIndex = 1;

for k = 1:data_length
    
    perStr = fix(k/step);
    str=['NavFilter Running ',num2str(perStr),'%'];
    waitbar(k/(data_length),hwait,str);
     
%% Read data
    [ acc,gyro,IMUtimestamp,dtIMU,dAngIMU,dVelIMU ] = readIMUData( accel,angRate,IMUframe,IMUtime );
    
    [ VelNED,PosNE,LatDeg,GPSAlt,GPS_DataArrived,gpslog ] = readGpsData( IMUtimestamp,GPSframe,GPStime,GPSStatus,GPSNsats,GPSLatDeg,GPSLngDeg,GPSHgt,GndSpd,CourseDeg,VelD,ekf_VNED);

    [ magData,magBias,MAG_DataArrived ] = readMagData( IMUtimestamp,MAGframe,MAGtime,mag,mag_bias );
    
    [ HgtMea,HGT_DataArrived ] = readHgtData( IMUtimestamp,BARO,GPSAlt );
    
    if(statesInitialised == 0 && GPS_DataArrived == 1)      %未初始化

        states(1:3) = [0;0;0];
        states(4:6) = ekf_VNED(k,:);         % North, East, Down velocity relative to and earth fixed reference point (m/s)
        states(7:9) = [0;0;0];                % North, East, Down position relative to and earth fixed reference point (m)
        states(10:12) = [0;0;0]; 
        states(13:15) = [0;0;0]; 
        InitEul = [mean(Roll(10:100)),mean(Pitch(10:100)),mean(Yaw(10:100))];
        quat = EulToQuat(InitEul);

        statesInitialised = 1;
    end

%% EKF
    if(statesInitialised == 1)      %已初始化
        Q = 1;
      % predict states
        [quat, states, Tbn, delAng, delVel]  = PredictStates(quat,states,gyro,acc,dtIMU,Q);
      % predict covariance matrix
        covariance  = PredictCovariance(delAng, delVel,quat,states,covariance,dtIMU);
        if( VelNED(1)^2+VelNED(2)^2 < 2^2)
                % Inhibit Acc bias state updates and covariance growth
                covariance(13:15,:) = single(0);
                covariance(:,13:15) = single(0);
            end
      % fuse velocity measurements - use synthetic measurements
%       GPS_DataArrived = 1;
      if(GPS_DataArrived==1)
        A1 = [A1;VelNED(1:3).' acc(1:3).' gyro(1:3).' IMUmsec];
        VelNED_ref = [VelNED_ref;VelNED(1:3).',IMUmsec];
        PosNED_ref = [PosNED_ref;PosNE(1:2).',HgtMea,GPSAlt,IMUmsec];
        
        measVel = [VelNED;PosNE;-HgtMea];
         [quat,states,angErr,covariance,velInnov,velInnovVar] = FuseVelocity(quat,states,covariance,measVel);
      end
      
      magBody = magData';
      magIndex = magIndex + 1;

      time = 1;
%       if (time >= 1.0 && headingAligned==0 && angErr < 1e-3)
%           quat = AlignHeading(quat,magBody,measDec);
           headingAligned = 1;
%       end
      % fuse magnetometer measurements if new data available and when tilt has settled
      if (headingAligned == 1)
          [quat,states,covariance,decInnov,decInnovVar] = FuseMagnetometer(quat,states,covariance,magBody,measDec,Tbn);
          decInnovLog(1,magIndex) = time;
          decInnovLog(2,magIndex) = decInnov;
          decInnovVarLog(1,magIndex) = time;
          decInnovVarLog(2,magIndex) = decInnovVar;
      end
    end

    time_est(k) = IMUmsec;
    eul = QuatToEul(quat);
    roll_est(k) = eul(1)*180/pi;
    pitch_est(k) = eul(2)*180/pi;
    yaw_est(k) = eul(3)*180/pi;
    
    velN_est(k) = states(4);
    velE_est(k) = states(5);
    velD_est(k) = states(6);
 
    posN_est(k) = states(7);
    posE_est(k) = states(8);
    posD_est(k) = -states(9);
    
    gyro_bais(k,:) = states(10:12);
    acc_bais(k,:) = states(13:15);
end

close(hwait);