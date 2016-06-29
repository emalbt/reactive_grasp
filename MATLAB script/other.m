Fstop = 5;
Fpass = 30;
Astop = 150;
Apass = 0.05;
Fs = 1/0.012;

d = designfilt('highpassfir','StopbandFrequency',Fstop, ...
  'PassbandFrequency',Fpass,'StopbandAttenuation',Astop, ...
  'PassbandRipple',Apass,'SampleRate',Fs,'DesignMethod','equiripple');


s1_fil = filter(d,imu0(:,axis));
 
figure(2);
subplot(2,1,1);
plot(imu0(:,axis));
hold on
grid on
subplot(2,1,2);
plot(s1_fil(:,1));
hold on
grid on
