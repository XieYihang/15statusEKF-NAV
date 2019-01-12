clear all;
addpath(genpath(pwd));

 load('160.BIN-357612.mat')
%load('219.BIN-125422.mat')
%load('27.BIN-633420.mat')

%% IMU Data


IMUframe = IMU(:,1);
IMUtime  = IMU(:,2)*1e-6;           % s
angRate  = IMU(:,3:5);              % rad/s
accel    = IMU(:,6:8);              % m/s^2
gyro_error = IMU(:,9);
accel_error = IMU(:,10);
gyro_health = IMU(:,12);
accel_health = IMU(:,13);


%% GPS Data

if ~isempty(GPS)
    goodDataIndices=find(GPS(:,3) >= 3);
    GPSframe = GPS(goodDataIndices,1);
    GPStime = GPS(goodDataIndices,2)*1e-6;
    GPSStatus = GPS(goodDataIndices,3);
    GPSNsats = GPS(goodDataIndices,6);
    GPSHdop = GPS(goodDataIndices,7);
    GPSLatDeg = GPS(goodDataIndices,8);
    GPSLngDeg = GPS(goodDataIndices,9);
    GPSHgt = GPS(goodDataIndices,10);               %   GPS相对高度  m
    GPSseaHgt = GPS(goodDataIndices,11);            %   GPS海拔高度
    deg2rad = pi/180;
    GndSpd = GPS(goodDataIndices,12);                %   m/s
    CourseDeg = GPS(goodDataIndices,13);             %
    VelD = GPS(goodDataIndices,14);
    
end

%% Magnetometer Data

if ~isempty(MAG)
    MAGframe = MAG(:,1);
    MAGtime  = MAG(:,2)*1e-6;
    mag      = MAG(:,3:5);              % 3~5:磁强计X,Y,Z
%     MagX     = MAG(:,3) - MAG(:,6);
%     MagY     = MAG(:,4) - MAG(:,7);
%     MagZ     = MAG(:,5) - MAG(:,8);
    mag_bias = MAG(:,6:8);              % 6~8:mag_offsetX,Y,Z          
%     MagBiasX = - MAG(:,6);
%     MagBiasY = - MAG(:,7);
%     MagBiasZ = - MAG(:,8);
    mag_motor_offset = MAG(:,9:11);     % 9~11:mag_motor_offsetX,Y,Z
%     Motor_offset_X = MAG(:,9);
%     Motor_offset_Y = MAG(:,10);
%     Motor_offset_Z = MAG(:,11);
    mag_health = MAG(:,12);             
end

%% Baro Data

if ~isempty(BARO)
    Baroframe = BARO(:,1);           % lineNO
    Barotime  = BARO(:,2)*1e-6;      % 
    BaroHgt   = BARO(:,3);           % 气压相对高度Alt      m
end

%% Reference Data

if ~isempty(ATT)
    ATTframe = ATT(:,1);
    ATTtime  = ATT(:,2)*1e-6;
    DesRoll  = ATT(:,3);            %期望roll deg
    Roll     = ATT(:,4)*deg2rad;    %实际控制用roll rad/s
    DesPitch  = ATT(:,5);
    Pitch    = ATT(:,6)*deg2rad;
    DesYaw   = ATT(:,7);
    Yaw      = ATT(:,8)*deg2rad;
    for k=1:size(Yaw)
        u1 = Yaw(k);
        if u1>pi
            u1 = u1-2*pi;
        else
            u1 = u1;
        end
        Yaw(k) = u1;
    end
    
    ErrRP    = ATT(:,9);            %   DCM姿态计算roll,pitch的校正误差，误差较大或DCM刚开始不久时不对EKF进行状态初始化    
    ErrYaw    = ATT(:,10);          %   DCM计算的yaw校正误差
end

if ~isempty(EKF1)
    EKFframe = EKF1(:,1);
    EKFtime  = EKF1(:,2)*1e-6;
    ekf_att = EKF1(:,3:5)*deg2rad;        %四元数变换而来 rad/s   3~5:roll,pitch,yaw
%     ekf_pitch = EKF1(:,4)*deg2rad; 
%     ekf_yaw   = EKF1(:,5)*deg2rad;
    ekf_VNED    = EKF1(:,6:8);               % m/s     6~8:VN,VE,VD
%     ekf_VE    = EKF1(:,7);
%     ekf_VD    = EKF1(:,8);
    ekf_PNED    = EKF1(:,9:11);               % m      9~11:PN,PE,PD
%     ekf_PE    = EKF1(:,10);
%     ekf_PD    = EKF1(:,11);
    ekf_gyroBiasXYZ  = EKF1(:,12:14);        % rad/s   12~14：X,Y,Z
%     ekf_gyroBiasY  = EKF1(:,13);
%     ekf_gyroBiasZ  = EKF1(:,14);
end

%% Save to files
save('NavFilter15stateData.mat', ...
    'IMUframe','IMUtime','angRate','accel','gyro_error','accel_error', 'gyro_health','accel_health',...
    'GPSframe','GPStime','GPSStatus','GPSNsats','GPSHdop','GPSLatDeg','GPSLngDeg','GPSHgt','GPSseaHgt','GndSpd','CourseDeg','VelD', ...
    'MAGframe','MAGtime','mag','mag_bias','mag_motor_offset','mag_health', ...
     'Baroframe','Barotime','BaroHgt','BARO',... 
    'ATTframe','ATTtime','DesRoll','Roll','DesPitch','Pitch','DesYaw','Yaw','ErrRP','ErrYaw', ...
     'EKFframe','EKFtime','ekf_att','ekf_VNED','ekf_PNED','ekf_gyroBiasXYZ');

clear all;
load('NavFilter15stateData.mat');
alignTime = min(IMUtime(IMUframe>GPSframe(find(GndSpd  >0.08, 1 )))) - 10;
startTime = alignTime - 30;

endTime = max(IMUtime)-1;
msecVelDelay = 300;             %毫秒
msecPosDelay = 300;
msecHgtDelay = 60;
msecMagDelay = 40;
msecTasDelay = 240;
EAS2TAS = 1.0;
