%% Plot states and state differences
close all;
load("state_diffs.mat");
dzs = state_diffs(3,:);
xs_mat = states_matlab(4,:);
xs_mpc = states_mpc(4,:);
figure(1);
subplot(2,1,1);
hold on;
plot(0.01*(1:length(dzs)),dzs);
ylabel("Difference (m)");   
subplot(2,1,2);
hold on;
plot(0.01*(1:length(xs_mat)),xs_mat,"DisplayName","current state");
plot(0.01*(1:length(xs_mpc)),xs_mpc,"DisplayName","state for mpc");
ylabel("Z Position (m)");
xlabel("Time (s)");
legend("Location","eastoutside");
%% Plot publishing rates
close all;
T = table2array(readtable('../debug_file.csv'));
times = T(:,1);
dts = [];
for i = 1:length(times)-1
   dts =  [dts times(i+1) - times(i)];
end
freqs = ones(1,length(dts))./dts;
figure(1);
hold on;
% plot(freqs,"o");
histogram(freqs,"Normalization","probability");
title("Publishing frequency for mpc\_control node");
xlabel("Publishing Frequency (Hz)");
ylabel("Relative Frequency");