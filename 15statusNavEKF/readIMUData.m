function [ acc,gyro,IMUtimestamp,dtIMU,dAngIMU,dVelIMU ] = readIMUData( accel,angRate,IMUframe,IMUtime )
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
% APM�㷨�л��Ϊ��������ģʽ��˫������ģʽ������������������ݶ������ͻ�ȡ��ֵ��Ϊ�������
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

