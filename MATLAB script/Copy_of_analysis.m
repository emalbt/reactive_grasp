

FolderName = '../0_Z_neg_B/';
File_ACC_raw    = fullfile(FolderName, 'ACC_raw.dat');
File_ACC_filt   = fullfile(FolderName, 'ACC_filt.dat');
% File_gyro_raw       = fullfile(FolderName, 'ACC_raw.dat');

ACC_raw  = load(File_ACC_raw); 
ACC_filt = load(File_ACC_filt); 


imu = 0;
ACC_raw_0 = select_imu(ACC_raw,imu);
ACC_filt_0 = select_imu(ACC_filt,imu);


hFig = figure(2);
set(hFig, 'Position', [500 500 2000 800])
subplot(2,1,1);
plot(ACC_raw_0(:,1));
hold on
plot(ACC_raw_0(:,2));
plot(ACC_raw_0(:,3));
legend('x','y','z','Location','southwest');

grid on
subplot(2,1,2);
plot(ACC_filt_0(:,1));
hold on
plot(ACC_filt_0(:,2));
plot(ACC_filt_0(:,3));
legend('x','y','z','Location','southwest');
grid on

