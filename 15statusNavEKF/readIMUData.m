function [ acc,gyro,IMUtimestamp,dtIMU,dAngIMU,dVelIMU ] = readIMUData( accel,angRate,IMUframe,IMUtime )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
% APM算法中会分为单传感器模式与双传感器模式，如果两个传感器数据都健康就会取均值作为变量输出
%

global IMUmsec;

persistent imuIndex;
if isempty(imuIndex)
    imuIndex = 2;
end

persistent prevAngRateIMU;
if isempty(prevAngRateIMU)
    prevAngRateIMU = single([0;0;0]);
end

persistent prevAccelIMU;
if isempty(prevAccelIMU)
    prevAccelIMU = single([0;0;0]);
end

if imuIndex>size(IMUframe)
    imuIndex =  size(IMUtime,1);
end

IMUtimestamp = IMUframe(imuIndex);
IMUmsec = IMUtime(imuIndex);
dtIMU = IMUtime(imuIndex) - IMUtime(imuIndex-1);
acc = accel(imuIndex,:)';
gyro = angRate(imuIndex,:)';

dAngIMU = 0.5*dtIMU*(gyro + prevAngRateIMU);
dVelIMU = 0.5*dtIMU*(acc   + prevAccelIMU  );

imuIndex = imuIndex+1;
prevAngRateIMU = gyro;
prevAccelIMU   = acc;

end

