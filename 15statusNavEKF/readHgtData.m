function [ HgtMea,HGT_DataArrived ] = readHgtData( IMUtimestamp,BARO,GPSAlt )
%UNTITLED5 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

persistent LastHgtMea;
if isempty(LastHgtMea)
    LastHgtMea = 1;
end

persistent barIndex;
if isempty(barIndex)
    barIndex = 1;
end

% persistent Barmsec;
% if isempty(Barmsec)
%     Barmsec = 0;
% end

persistent LastBARmsec;
if isempty(LastBARmsec)
    LastBARmsec = 0;
end

persistent BARtimestamp;
if isempty(BARtimestamp)
    BARtimestamp = BARO(1,1);
end

persistent nextBARtimestamp;
if isempty(nextBARtimestamp)
    nextBARtimestamp = BARO(1,1);%;0
end

HgtMea = single(0.0);

while ( nextBARtimestamp < IMUtimestamp )
    index = barIndex;
    if index+1>size(BARO,1)
        break;
    end
    barIndex = barIndex + 1;
    
    nextBARtimestamp = BARO(index+1,1);
    BARtimestamp = BARO(index,1);
    Barotime  = BARO(index,2)*1e-6;      % 
    BaroHgt   = BARO(index,3);           % ��ѹ��Ը߶�Alt      m
%     gpsAlt    = GPSHgt(index);
    
 %   HgtMea = GPSAlt;            %��gps�߶���Ϊ�߶�������
   HgtMea = BaroHgt;
    

%         %���û����ɣ������ѹ���˲���ȡһ���Զ���ɵ���Ը߶ȣ�it is is reset to last height measurement on disarming in performArmingChecks()
%         if (!getTakeoffExpected()) 
%             gndHgtFiltTC = 0.5;
%             dtBaro = msecHgtAvg/1000;
%             alpha = constrain_float(dtBaro / (dtBaro+gndHgtFiltTC),0.0,1.0);
%             meaHgtAtTakeOff = meaHgtAtTakeOff + (hgtMea-meaHgtAtTakeOff)*alpha;
%          
%         else
%             %����������Զ����ģʽ���߶����������ᱻ���ơ������������ֹ�������ϴ������ĸ���ѹ���������׶ε�EKF�߶�
%             if (vehicleArmed && getTakeoffExpected()) 
%                 hgtMea = max(hgtMea, meaHgtAtTakeOff);
%             end
%         end

end

if(BARtimestamp>LastBARmsec)         
    LastBARmsec = BARtimestamp;
    LastHgtMea = HgtMea;
    HGT_DataArrived = 1;
else
    HgtMea= LastHgtMea;
    HGT_DataArrived = 0;
end


end

