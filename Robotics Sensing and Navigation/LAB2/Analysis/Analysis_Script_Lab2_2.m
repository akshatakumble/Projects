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
            time = str2double(fields{2});
            lat = str2double(fields{3});
            latDir = fields{4};
            lon = str2double(fields{5});
            lonDir = fields{6};
            fixQuality = str2double(fields{7});
            hdop = str2double(fields{9});
            altitude = str2double(fields{10});
            
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
            
            tempData = [tempData; time, E, N, altitude, hdop];
        end
    end
    fclose(fid);
    data{i} = tempData;
end

% Perform analysis for each file separately
for i = 1:length(files)
    dataset = data{i};
    
    if isempty(dataset)
        continue;
    end
    
    figure;
    
    % Normalize to first point
    E0 = dataset(1,2);
    N0 = dataset(1,3);
    
    % Scatter plot of Northing vs Easting
    subplot(2,2,1);
    scatter(dataset(:,2) - E0, dataset(:,3) - N0, 10, 'filled');
    xlabel('Easting (m)'); ylabel('Northing (m)');
    title(['Stationary Scatter Plot - File ', num2str(i)]);
    axis equal; grid on;
    
    % Compute and display errors for stationary data
    errors = sqrt((dataset(:,2) - E0).^2 + (dataset(:,3) - N0).^2);
    avg_error = mean(errors);
    max_error = max(errors);
    
    fprintf('File %d - Stationary Data Errors:\n', i);
    fprintf('  Mean Error: %.4f m\n', avg_error);
    fprintf('  Max Error: %.4f m\n\n', max_error);
    
    % Histogram of position error
    subplot(2,2,2);
    histogram(errors, 20);
    xlabel('Position Error (m)'); ylabel('Frequency');
    title(['Error Histogram - File ', num2str(i)]);
    
    % Altitude vs. Time    
    subplot(2,2,3);
    plot(dataset(:,1), dataset(:,4), 'b-');
    xlabel('Time (s)'); ylabel('Altitude (m)');
    title(['Altitude vs Time - File ', num2str(i)]);
    grid on;
    
    % Path analysis for moving datasets
    if mod(i, 2) == 0 % Only for moving datasets (even files)
        subplot(2,2,4);
        
        % Normalize to first point
        E0 = dataset(1,2);
        N0 = dataset(1,3);
        
        % Scatter plot of trajectory
        plot(dataset(:,2) - E0, dataset(:,3) - N0, '-o');
        xlabel('Easting (m)'); ylabel('Northing (m)');
        title(['Moving Rover Path - File ', num2str(i)]);
        axis equal; grid on;
        hold on;
        
        % Define the points for File 2 and File 4
        if i == 2
            % Points for File 2
            points = [
                42.3381349, -71.0869355;
                42.3376807, -71.0861906;
                42.3377528, -71.0860883;
                42.3382306, -71.0868169;
                42.3381349, -71.0869355
            ];
        elseif i == 4
            % Points for File 4
            points = [
                42.338866, -71.086800;
                42.339480, -71.087241;
                42.339711, -71.086586;
                42.339280, -71.086276;
                42.338866, -71.086800
            ];
        else
            continue;
        end
        
        % Convert the points to UTM coordinates
        utmStruct = defaultm('utm');
        utmZone = utmzone(points(1,1), points(1,2));
        utmStruct.zone = utmZone;
        utmStruct.geoid = wgs84Ellipsoid;
        utmStruct = defaultm(utmStruct);
        [E_points, N_points] = projfwd(utmStruct, points(:,1), points(:,2));
        
        % Compute errors for moving data
        moving_errors = [];
        
        % Plot best fit lines for each pair of consecutive points
        for j = 1:length(points)-1
            x_data = E_points(j:j+1);
            y_data = N_points(j:j+1);
            
            % Fit a line to the points
            p = polyfit(x_data, y_data, 1); % Linear fit
            fittedY = polyval(p, x_data);
            
            % Plot the best fit line
            plot(x_data - E0, fittedY - N0, 'r-', 'LineWidth', 2);
            
            % Compute distance of actual points from the line
            for k = 1:size(dataset,1)
                x = dataset(k,2);
                y = dataset(k,3);
                
                % Perpendicular distance from point (x,y) to line 
                y = p(1)*x + p(2);
                error_dist = abs(p(1) * x - y + p(2)) / sqrt(p(1)^2 + 1);
                moving_errors = [moving_errors; error_dist];
            end
        end
        
        % Print moving errors
        avg_moving_error = mean(moving_errors);
        max_moving_error = max(moving_errors);
        
        fprintf('File %d - Moving Data Errors:\n', i);
        fprintf('  Mean Error: %.4f m\n', avg_moving_error);
        fprintf('  Max Error: %.4f m\n\n', max_moving_error);
        
        hold off;
    end
end

% Helper function
function decimal = dms2decimal(dms, direction)
    degrees = floor(dms / 100);
    minutes = mod(dms, 100);
    decimal = degrees + minutes / 60;
    
    if direction == 'S' || direction == 'W'
        decimal = -decimal;
    end
end
