clear all 
close all

FolderName = '../logs/20160608_164628/';
File_ACC_raw    = fullfile(FolderName, 'ACC_raw.dat');
File_ACC_filt   = fullfile(FolderName, 'ACC_filt.dat');
% File_gyro_raw       = fullfile(FolderName, 'ACC_raw.dat');

ACC_raw  = load(File_ACC_raw); 
ACC_filt = load(File_ACC_filt); 


imu = 9;
ACC_raw_0 = select_imu(ACC_raw,imu);
ACC_filt_0 = select_imu(ACC_filt,imu);

f = 120;      %first sample
l = f + 44;
X = [ACC_filt_0(f:l,1), ACC_filt_0(f:l,2), ACC_filt_0(f:l,3)];

vec = X(1,:);
for(i=2:length(X))
    tmp =  X(i,:);
    vec = [vec,tmp];
end

% figure(1);
hFig = figure(1);
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

