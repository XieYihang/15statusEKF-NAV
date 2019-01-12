%% Attitude
fileName = 'EulerAngleEstimates';
figure;
hold on
plot(time_est,roll_est,'b');
plot(time_est,Roll(1:data_length)'*180/pi,'g');
hold off
grid on;
ylim([-200 200]);
xlabel('time (sec)');ylabel('roll (deg)');
title('Roll Estimates');

figure;
hold on
plot(time_est,pitch_est,'b');
plot(time_est,Pitch(1:data_length)'*180/pi,'g');
hold off
grid on;
ylim([-200 200]);
xlabel('time (sec)');ylabel('pitch (deg)');
title('Pitch Estimates');

figure;
hold on
plot(time_est,yaw_est,'b');
plot(time_est,Yaw(1:data_length)'*180/pi,'g');
hold off
grid on;
ylim([-200 200]);
xlabel('time (sec)');ylabel('yaw (deg)');
title('Yaw Estimates');

%% NED velocity
figure;
hold on
plot(time_est,velN_est,'b');
plot(time_est,ekf_VNED(1:data_length,1)','g');
plot(A1(:,10),A1(:,1),'r');
hold off
grid on;
xlabel('time (sec)');ylabel('North Velocity (m/s)');
title('NED Velocity Estimates');

figure;
hold on
plot(time_est,velE_est,'b');
plot(time_est,ekf_VNED(1:data_length,2)','g');
plot(A1(:,10),A1(:,2),'r');
hold off
grid on;
xlabel('time (sec)');ylabel('East Velocity (m/s)');

figure;
hold on
plot(time_est,velD_est,'b');
plot(time_est,ekf_VNED(1:data_length,3)','g');
plot(A1(:,10),A1(:,3),'r');
hold off
grid on;
xlabel('time (sec)');ylabel('Down Velocity (m/s)');

%% Gyro Bias
figure;
subplot(3,1,1);
plot(time_est(:),gyro_bais(:,1));
grid on;
xlabel('time (sec)');ylabel('X (Deg/s)');
title('Gyro Bias Error Estimates');
subplot(3,1,2);
plot(time_est(:),gyro_bais(:,2));
grid on;
xlabel('time (sec)');ylabel('Y (Deg/s)');
subplot(3,1,3);
hold on
plot(time_est(:),gyro_bais(:,3),'k');
% plot(time_est(:),ekf_gyroBiasXYZ(1:data_length,3),'r');
hold off
grid on;
xlabel('time (sec)');ylabel('Z (Deg/s)');


%% NE Position and Height
figure;
hold on
plot(time_est,posN_est,'b');
% plot(time_est,ekf_PNED(1:data_length,1)','g');
plot(PosNED_ref(:,5),PosNED_ref(:,1),'r');
hold off
grid on;
xlabel('time (sec)');ylabel('North (m)');
title('NED Position Estimates');

figure;
hold on
plot(time_est,posE_est,'b');
% plot(time_est,ekf_PNED(1:data_length,2)','g');
plot(PosNED_ref(:,5),PosNED_ref(:,2),'r');
hold off
grid on;
xlabel('time (sec)');ylabel('East (m)');

figure;
hold on
plot(time_est,posD_est,'b');
% plot(time_est,ekf_PNED(1:data_length,3)','g');
plot(PosNED_ref(:,5),PosNED_ref(:,3),'r');
hold off
grid on;
xlabel('time (sec)');ylabel('Down (m)');


%% Accelerometer Bias
figure;
subplot(3,1,1);
plot(time_est(:),acc_bais(:,1));
grid on;
xlabel('time (sec)');ylabel('X (m/s^2)');
title('Acc Bias Error Estimates');
subplot(3,1,2);
plot(time_est(:),acc_bais(:,2));
grid on;
xlabel('time (sec)');ylabel('Y (m/s^2)');
subplot(3,1,3);
hold on
plot(time_est(:),acc_bais(:,3),'k');
hold off
grid on;
xlabel('time (sec)');ylabel('Z (m/s^2)');


%% Velocity Innovations
% figure;
% subplot(3,1,1);
% plot(time1(:),innovVelN(:));
% grid on;
% xlabel('time (sec)');ylabel('North (m/s)');
% title('Velocity Measurement Innovations');
% subplot(3,1,2);
% plot(time1(:),innovVelE(:));
% grid on;
% xlabel('time (sec)');ylabel('East (m/s)');
% subplot(3,1,3);
% plot(time1(:),innovVelD(:));
% grid on;
% xlabel('time (sec)');ylabel('Down (m/s)');
