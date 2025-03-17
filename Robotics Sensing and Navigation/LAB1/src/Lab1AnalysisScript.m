
clc;
clear all;
close all;

ros2genmsg('C:\Users\aksha\Downloads\Robotics')
bagReader = ros2bagreader('C:\Users\aksha\Downloads\Robotics\Data\Data\stat1a\stat1a_0.db3');

%%

statData='C:\Users\aksha\Downloads\Robotics\Data\Data\stat1a\stat1a_0.db3';
buildData='C:\Users\aksha\Downloads\Robotics\Data\Data\occa1\occa1_0.db3';
walkData='C:\Users\aksha\Downloads\Robotics\Data\Data\walking\walking_0.db3';

stationaryBag=ros2bag(statData);
buildingBag=ros2bag(buildData);
walkData=ros2bag(walkData);

msgs=readMessages(select(stationaryBag,'Topic','/gps'));
msgsB=readMessages(select(buildingBag,'Topic','/gps'));
msgsW=readMessages(select(walkData,'Topic','/gps'));

% Stationary open data
numMsgs=length(msgs);
lat=zeros(numMsgs,1);
lon=zeros(numMsgs,1);
eastings=zeros(numMsgs,1);
northings=zeros(numMsgs,1);
alt=zeros(numMsgs,1);
hdop=zeros(numMsgs,1);
timestamp=zeros(numMsgs,1);
for i=1:numMsgs
    lat(i)=msgs{i}.latitude;
    lon(i)=msgs{i}.longitude;
    eastings(i)=msgs{i}.utm_easting;
    northings(i)=msgs{i}.utm_northing;
    alt(i)=msgs{i}.altitude;
    hdop(i)=msgs{i}.hdop;
    timestamp(i)=double(msgs{i}.header.stamp.sec);
end

%Stationary occluded data (with buildings)
numMsgsB=length(msgsB);
latB=zeros(numMsgsB,1);
lonB=zeros(numMsgsB,1);
eastingsB=zeros(numMsgsB,1);
northingsB=zeros(numMsgsB,1);
altB=zeros(numMsgsB,1);
hdopB=zeros(numMsgsB,1);
timestampB=zeros(numMsgsB,1);
for i=1:numMsgsB
    latB(i)=msgsB{i}.latitude;
    lonB(i)=msgsB{i}.longitude;
    eastingsB(i)=msgsB{i}.utm_easting;
    northingsB(i)=msgsB{i}.utm_northing;
    altB(i)=msgsB{i}.altitude;
    hdopB(i)=msgsB{i}.hdop;
    timestampB(i)=double(msgsB{i}.header.stamp.sec);
end


% Walking data
numMsgsW=length(msgsW);
latW=zeros(numMsgsW,1);
lonW=zeros(numMsgsW,1);
eastingsW=zeros(numMsgsW,1);
northingsW=zeros(numMsgsW,1);
altW=zeros(numMsgsW,1);
hdopW=zeros(numMsgsW,1);
timestampW=zeros(numMsgsW,1);
for i=1:numMsgsW
    latW(i)=msgsW{i}.latitude;
    lonW(i)=msgsW{i}.longitude;
    eastingsW(i)=msgsW{i}.utm_easting;
    northingsW(i)=msgsW{i}.utm_northing;
    altW(i)=msgsW{i}.altitude;
    hdopW(i)=msgsW{i}.altitude;
    timestampW(i)=double(msgsW{i}.header.stamp.sec);
end



figure;
plot(alt);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude vs Time (Stationary Open Area Data)');
grid on;

figure;
plot(altB);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude vs Time (Occluded Data)');
grid on;


% % Normalize to the first value
northings1 = northings - northings(1);
eastings1 = eastings - eastings(1);
northingsB1 = northingsB - northingsB(1);
eastingsB1 = eastingsB - eastingsB(1);
northingsW1 = northingsW - northingsW(1);
eastingsW1 = eastingsW - eastingsW(1);

% % Plot for open stationary data
figure;
scatter(eastings1, northings1, 10, 'b', 'filled');
xlabel('Easting (m)');
ylabel('Northing (m)');
title('Scatterplot of Northing vs Easting (Open Area)');
grid on;
axis equal;


[e1, n1, zone] = deg2utm(latB, lonB);
% Plot for stationary occluded data (with buildings)
figure;
scatter(e1, n1, 10, 'r', 'filled');
xlabel('Easting (m)');
ylabel('Northing (m)');
title('Scatterplot of Northing vs Easting (Surrounded by Buildings)');
grid on;
axis equal;

% Known reference point (replace with actual values if available
ref_lat = 42.340876;
ref_long = -71.097120;

ref_latB = 42.338954;
ref_longB = -71.090639;

% % Compute error as Euclidean distance
[eKnown1, nKnown1, z1] = deg2utm(ref_lat, ref_long);
[eKnown2, nKnown2, z2] = deg2utm(ref_latB, ref_longB);
error_stationary = sqrt((northings - nKnown1).^2 + (eastings - eKnown1).^2);
error_stationaryB = sqrt((n1 - nKnown2).^2 + (e1 - eKnown2).^2);
rmse = sqrt(mean(error_stationary.^2));
fprintf('RMSE stationary: %.2f m\n', rmse);
rmse = sqrt(mean(error_stationaryB.^2));
fprintf('RMSE occluted: %.2f m\n', rmse);


% % Plot histograms
figure;
histogram(error_stationary, 20);
xlabel('Error Distance (m)');
ylabel('Frequency');
title('Error Histogram - Open Stationary Data');
grid on;

figure;
histogram(error_stationaryB, 20);
xlabel('Error Distance (m)');
ylabel('Frequency');
title('Error Histogram - Stationary Data with Buildings');
grid on;

% % Compute Mean Error
mean_error_stationary = mean(error_stationary);
mean_error_stationaryB = mean(error_stationaryB);
disp(['Mean Error (Open Area): ', num2str(mean_error_stationary), ' meters']);
disp(['Mean Error (With Buildings): ', num2str(mean_error_stationaryB), ' meters']);

disp(mean(hdop))
disp(mean(hdopB))

% %% % Normalize time
timeW = timestampW - timestampW(1);

% % Plot Altitude vs Time
figure;
plot(timeW, altW, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude vs Time (Walking Data)');
grid on;


% Fit a linear model to the walking data
p = polyfit(eastingsW1, northingsW1, 1);
fit_line = polyval(p, eastingsW1);

% Compute error from best-fit line
error_walk = abs(northingsW1 - fit_line);

% Scatterplot with Best Fit Line
figure;
scatter(eastingsW1, northingsW1, 10, 'g', 'filled');
hold on;
plot(eastingsW1, fit_line, 'r', 'LineWidth', 1.5);
xlabel('Easting (m)');
ylabel('Northing (m)');
title('Walking Data with Line of Best Fit');
grid on;
legend('GPS Data', 'Best Fit Line');

% % Histogram of Error from Best Fit Line
figure;
histogram(error_walk, 20);
xlabel('Error Distance (m)');
ylabel('Frequency');
title('Error Histogram - Walking Data');
grid on;

% Compute Mean Error for Walking Data
mean_error_walk = mean(error_walk);
disp(['Mean Error (Walking Data): ', num2str(mean_error_walk), ' meters']);


