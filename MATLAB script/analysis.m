clear all 
close all

%% load file 
FolderName = '../logs/20161220_171215/';
File_ACC_raw    = fullfile(FolderName, 'ACC_raw.dat');
File_ACC_filt   = fullfile(FolderName, 'ACC_filt.dat');
% File_gyro_raw       = fullfile(FolderName, 'ACC_raw.dat');

ACC_raw  = load(File_ACC_raw); 
ACC_filt = load(File_ACC_filt); 


%% select imu 
imu = input('Enter the imu number [0 1 2 3 4 5 6]:   ');
ACC_raw_0 = select_imu(ACC_raw,imu);
ACC_filt_0 = select_imu(ACC_filt,imu);

% figure(1);
hFig = figure(1);
set(hFig, 'Position', [500 500 2000 800])

subplot(2,1,1);
plot(ACC_raw_0(:,1));
title(['raw data - imu  ', num2str(imu)]);
hold on
plot(ACC_raw_0(:,2));
plot(ACC_raw_0(:,3));
legend('x','y','z','Location','southwest');

grid on
subplot(2,1,2);
plot(ACC_filt_0(:,1));
title(['filtered data - imu  ', num2str(imu)]);
hold on
plot(ACC_filt_0(:,2));
plot(ACC_filt_0(:,3));
legend('x','y','z','Location','southwest');
grid on

%% select first sample 
f = input('select on the plot the first sample and insert it:   ');      
sl = 40; %signal length
l = f + sl;
X = [ACC_filt_0(f:l,1), ACC_filt_0(f:l,2), ACC_filt_0(f:l,3)];

vec = X(1,:);
for(i=2:length(X))
    tmp =  X(i,:);
    vec = [vec,tmp];
end

%% write on file
tofile

figure(3)
plot(X);
title(['selected signal for imu ', num2str(imu), '  at sample ', num2str(f)]);
grid on
legend('x','y','z','Location','southwest');
