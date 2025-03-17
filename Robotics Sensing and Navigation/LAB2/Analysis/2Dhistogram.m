clc; clear; close all;

% Load Data
files = {
    "C:\Users\aksha\Downloads\open-stat-1.txt" ,
    "C:\Users\aksha\Downloads\open-square.txt", 
    "C:\Users\aksha\Downloads\occluded-stationary-issec.txt", 
    "C:\Users\aksha\Downloads\occluded-sqr.txt"
};
     
data = cell(length(files), 1);

for i = 1:length(files)
    fid = fopen(files{i}, 'r');
    tempData = [];
    while ~feof(fid)
        line = fgetl(fid);
        if startsWith(line, "$GNGGA")
            fields = strsplit(line, ',');
            lat = str2double(fields{3});
            latDir = fields{4};
            lon = str2double(fields{5});
            lonDir = fields{6};
            
            % Convert lat/lon to decimal degrees
            lat = dms2decimal(lat, latDir);
            lon = dms2decimal(lon, lonDir);
            
            % Convert lat/lon to UTM
            utmZone = utmzone(lat, lon); 
            utmStruct = defaultm('utm');  
            utmStruct.zone = utmZone;  
            utmStruct.geoid = wgs84Ellipsoid;  
            utmStruct = defaultm(utmStruct);  
            [E, N] = projfwd(utmStruct, lat, lon);  
            
            tempData = [tempData; E, N];
        end
    end
    fclose(fid);
    data{i} = tempData;
end

% Analyze UTM Data
for i = 1:length(files)
    dataset = data{i};
    
    if isempty(dataset)
        continue;
    end
    
    % Compute statistics
    meanE = mean(dataset(:,1));
    meanN = mean(dataset(:,2));
    stdE = std(dataset(:,1));
    stdN = std(dataset(:,2));
    
    % Print statistics
    fprintf('File %d:\n', i);
    fprintf('  Mean Easting: %.4f m, Std Dev: %.4f m\n', meanE, stdE);
    fprintf('  Mean Northing: %.4f m, Std Dev: %.4f m\n', meanN, stdN);
    
    % Plot 2D histogram
    figure;
    histogram2(dataset(:,1), dataset(:,2), 'Normalization', 'probability', ...
        'DisplayStyle', 'tile', 'ShowEmptyBins', 'on');
    colorbar;
    xlabel('Easting (m)'); ylabel('Northing (m)');
    title(['2D Histogram - File ', num2str(i)]);
    axis equal;
end

% Helper function to convert DMS to decimal degrees
function decimal = dms2decimal(dms, direction)
    degrees = floor(dms / 100);
    minutes = mod(dms, 100);
    decimal = degrees + minutes / 60;
    if direction == 'S' || direction == 'W'
        decimal = -decimal;
    end
end