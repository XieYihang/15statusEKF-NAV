function  [ magData,magBias,MAG_DataArrived ] = readMagData( IMUtimestamp,MAGframe,MAGtime,MAG,MAG_BIAS )
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明


persistent LastmagData;
if isempty(LastmagData)
    LastmagData = single([0;0;0]);
end

persistent LastmagBias;
if isempty(LastmagBias)
    LastmagBias = single([0;0;0]);
end

persistent magIndex;
if isempty(magIndex)
    magIndex = 1;
end

persistent MAGmsec;
if isempty(MAGmsec)
    MAGmsec = 0;
end

persistent LastMAGmsec;
if isempty(LastMAGmsec)
    LastMAGmsec = 0;
end

persistent MAGtimestamp;
if isempty(MAGtimestamp)
    MAGtimestamp = 0;
end


while ( MAGtimestamp < IMUtimestamp )
    index = magIndex;
    if magIndex>size(MAGframe,1)
        break;
    end
    magIndex = magIndex + 1;
    
    MAGtimestamp = MAGframe(index);
    MAGmsec = MAGtime(index);
    magData = 0.001*( MAG(index,:) - MAG_BIAS(index,:) );
    magBias = -0.001*MAG_BIAS(index,:);
    
% % check if compass offsets have ben changed and adjust EKF bias states to maintain consistent innovations
%         if (_ahrs->get_compass()->healthy()) {
%             Vector3f nowMagOffsets = _ahrs->get_compass()->get_offsets();
%             bool changeDetected = (!is_equal(nowMagOffsets.x,lastMagOffsets.x) || !is_equal(nowMagOffsets.y,lastMagOffsets.y) || !is_equal(nowMagOffsets.z,lastMagOffsets.z));
% % Ignore bias changes before final mag field and yaw initialisation, as there may have been a compass calibration
%             if (changeDetected && secondMagYawInit) {
%                 state.body_magfield.x += (nowMagOffsets.x - lastMagOffsets.x) * 0.001f;
%                 state.body_magfield.y += (nowMagOffsets.y - lastMagOffsets.y) * 0.001f;
%                 state.body_magfield.z += (nowMagOffsets.z - lastMagOffsets.z) * 0.001f;
%             }
%         lastMagOffsets = nowMagOffsets;
%         }
    
end

if(MAGmsec>LastMAGmsec)         
    LastMAGmsec = MAGmsec;
    LastmagData = magData;
    LastmagBias = magBias;
    MAG_DataArrived = 1;
else
    magData = LastmagData;
    magBias = LastmagBias;
    MAG_DataArrived = 0;
end

end

