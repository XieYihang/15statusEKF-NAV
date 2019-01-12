function  [ VelNED,PosNE,LatDegData,GPSAlt,GPS_DataArrived,gpslog ] = readGpsData( IMUtimestamp,GPSframe,GPStime,GPSStatus,GPSNsats,LatDeg,LngDeg,GPSHgt,GndSpd,CourseDeg,VelD,ekf_VNED )
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
 global IMUmsec;

 VelNED = single([0;0;0]);
 PosNE = single([0;0]);
 persistent Lastgpslog;
if isempty(Lastgpslog)
    Lastgpslog = 1;
end

persistent LastGPSAlt;
if isempty(LastGPSAlt)
    LastGPSAlt = single([0;0;0]);
end

persistent LastVelNED;
if isempty(LastVelNED)
    LastVelNED = single([0;0;0]);
end
persistent LastPosNE;
if isempty(LastPosNE)
    LastPosNE = single([0;0]);
end
persistent LastLatDegData;
if isempty(LastLatDegData)
    LastLatDegData = 0;
end
    
persistent gpsIndex;
if isempty(gpsIndex)
    gpsIndex = 1;
end

persistent GPSmsec;
if isempty(GPSmsec)
    GPSmsec = GPStime(1);
end
persistent LastGPSmsec;
if isempty(LastGPSmsec)
    LastGPSmsec = GPStime(1);
end

persistent GPStimestamp;
if isempty(GPStimestamp)
    GPStimestamp = GPSframe(1);%;0
end

persistent nextGPStimestamp;
if isempty(nextGPStimestamp)
    nextGPStimestamp = GPSframe(1);%;0
end


persistent RefLatLongDeg;
if isempty(RefLatLongDeg)
    RefLatLongDeg = [mean(LatDeg(10:100)),mean(LngDeg(10:100))];
end
validOrigin = 1;        %初始点有效标志位

deg2rad = single(pi/180);
earthRadius = single(6378145);

%% APM的log生成的mat文件每一条消息都有自己唯一的lineNO,这组序号是公用的,用此来做数据同步
while ( (nextGPStimestamp <= IMUtimestamp) )  
    
    index = gpsIndex;
    if gpsIndex>size(GPSframe)
        break;
    end
    gpsIndex = gpsIndex+1;
    if index>size(GPSframe,1)-1
        break;
    end
    
    if( GPSStatus(index) >= 3 )    %&&(IMUmsec - GPSmsec<0.2)
            nextGPStimestamp = GPSframe(index+1);
            GPStimestamp  = GPSframe(index); 
            GPSmsec = GPStime(index);
            gpsVelD = VelD(index);
          
%%
% use the speed accuracy from the gps if available,otherwise set to
% zero. Apply a decaying envelope filter with a 5s time constant to the raw
% speed accuracy data

%             alpha = constrain_float(0.0002*( GPStime(index-1)- GPStime(index-2)),0,1);
%             gpsSpdAccuracy = gpsSpdAccuracy*(1-alpha);
%             if( gps提供的速度精度为零 ){ gpsSpdAccuracy = 0 }            
%             else { gpsSpdAccuracy=max(当前计算速度精度,gps速度精度) }

            GPSsatsnum = GPSNsats(index);
            
            if(GPSsatsnum>=6)
                    gpsNoiseScaler = 1;
            else if(GPSsatsnum == 5)
                    gpsNoiseScaler = 1.4;
                else
                    gpsNoiseScaler = 2;
                end
            end
            
%%            
% monitor quality of thr gps velocity data for alignment 
% Monitor GPS data to see if quality is good enough to initialise the EKF
% Monitor magnetometer innovations to to see if the heading is good enough to use GPS
% Return true if all criteria pass for 10 seconds
%
%           [ goodToAlign ] = calcGpsGoodToAlign( gpsSpdAccuracy,gpsVelD,GPSstatus );
  

            %如果初始点有效，则计算位置偏差，若初始点无效，则初始化初始点
            if( validOrigin == 1 )
%                 validOrigin = 0;
                gpsCourse = CourseDeg(index);
                gpsGndSpd = GndSpd(index);
            
                gpsLat = LatDeg(index);
                gpsLon = LngDeg(index);
                GPSAlt = GPSHgt(index);
            
                LatLongDeg= [gpsLat,gpsLon];
                LatLongDelta = (LatLongDeg-RefLatLongDeg) * deg2rad;
                PosNE(1) = earthRadius * LatLongDelta(1);
                PosNE(2) = earthRadius * cos(RefLatLongDeg(1)*deg2rad) * LatLongDelta(2);
                VelNED(1) = gpsGndSpd*cos(gpsCourse*deg2rad);
                VelNED(2) = gpsGndSpd*sin(gpsCourse*deg2rad);
% VelNED(1) = ekf_VNED(index,1);
% VelNED(2) = ekf_VNED(index,2);
                VelNED(3) = gpsVelD;
            else
                %初始点未初始化，判断home点设置，条件，满足条件则进行设置
                if( goodToAlign == 1)   
                    RefLatLongDeg = [LatDeg(index),LngDeg(index)];
                    
%                     % Now we know the location we have an estimate for the magnetic field declination and adjust the earth field accordingly
%                     alignMagStateDeclination();
%                     % Set the height of the NED origin to eight of baro height datum relative to GPS height datum'
%                     EKF_origin.alt = gpsloc.alt - hgtMea;
%                     % 重新设置home点，因此实时的PosNE设为0
%                     gpsPosNE.zero();
%
%                     % If the vehicle is in flight (use arm status to determine) and GPS useage isn't explicitly prohibited, we switch to absolute position mode
%                     if (vehicleArmed && _fusionModeGPS != 3) 
%                         constPosMode = false;
%                         PV_AidingMode = AID_ABSOLUTE;
%                         gpsNotAvailable = false;
%                         % Initialise EKF position and velocity states
%                         ResetPosition();
%                         ResetVelocity();
%                     end

                end
            end
            %calculate a position offset which is applied to NE position and velocity wherever it is used throughout code to allow GPS position jumps to be accommodated gradually
            %衰减速度？
            %[ PosNE,VelNED ]=decayGpsOffset( IMUmsec );
            LatDegData = gpsLat * deg2rad;        
    end
    
end

% 如果之前GPS被上锁或禁用，或者EKF未初始化，则我们声明GPS是无效的
%     if ((_ahrs->get_gps().status() < AP_GPS::GPS_OK_FIX_3D) || _fusionModeGPS == 3 || !validOrigin) 
%         gpsNotAvailable = true;
%     else 
%         gpsNotAvailable = false;
%     end

%GPS数据更新，PS:在APM中GPS数据更新与GPS数据有效十分开判断的
if(GPSmsec>LastGPSmsec)         
    LastGPSmsec = GPSmsec;
    LastVelNED = VelNED;
    LastPosNE = PosNE;
    LastLatDegData = LatDegData;
    LastGPSAlt = GPSAlt;
%     GPSmsec
%     IMUmsec
%     IMUtimestamp
%     GPStimestamp
    gpslog = [IMUmsec,GPSmsec,IMUtimestamp,GPStimestamp];

    GPS_DataArrived = 1;
else
    VelNED = LastVelNED;
    PosNE = LastPosNE;
    LatDegData = LastLatDegData;
    GPSAlt = LastGPSAlt;
    gpslog = Lastgpslog;
    GPS_DataArrived = 0;
end
    
    
end

