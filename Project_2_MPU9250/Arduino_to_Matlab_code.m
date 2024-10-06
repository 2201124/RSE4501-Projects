% Initialize serial port
port = "COM3";
baudrate = 115200;
s = serialport(port, baudrate);
flush(s);

% Initialize animated lines for real-time plotting
figure;
subplot(2, 1, 1);
accel_x_line = animatedline('Color', 'r'); % Red for X-axis
accel_y_line = animatedline('Color', 'g'); % Green for Y-axis
accel_z_line = animatedline('Color', 'b'); % Blue for Z-axis
title('Accelerometer Data');
xlabel('Time (s)');
ylabel('Acceleration');
legend('Accel X', 'Accel Y', 'Accel Z');
hold on;

subplot(2, 1, 2);
gyro_x_line = animatedline('Color', 'r'); % Red for X-axis
gyro_y_line = animatedline('Color', 'g'); % Green for Y-axis
gyro_z_line = animatedline('Color', 'b'); % Blue for Z-axis
title('Gyroscope Data');
xlabel('Time (s)');
ylabel('Angular Velocity');
legend('Gyro X', 'Gyro Y', 'Gyro Z');
hold on;

% Set up a loop to read data and plot
start_time = datetime('now'); % Starting time for the X-axis

while true
    if s.NumBytesAvailable > 0
        % Get data from the getData function
        sensor_values = getData(s);

        if length(sensor_values) == 6
            % Calculate elapsed time for X-axis
            elapsed_time = seconds(datetime('now') - start_time);

            % Update accelerometer lines
            addpoints(accel_x_line, elapsed_time, sensor_values(1));
            addpoints(accel_y_line, elapsed_time, sensor_values(2));
            addpoints(accel_z_line, elapsed_time, sensor_values(3));

            % Update gyroscope lines
            addpoints(gyro_x_line, elapsed_time, sensor_values(4));
            addpoints(gyro_y_line, elapsed_time, sensor_values(5));
            addpoints(gyro_z_line, elapsed_time, sensor_values(6));

            % Update the plots and limit the time range to 10 seconds
            if elapsed_time > 10
                x_limits = [elapsed_time - 10, elapsed_time]; % Show the last 10 seconds
            else
                x_limits = [0, 10]; % Show from 0 to 10 seconds initially
            end

            subplot(2, 1, 1);
            xlim(x_limits);
            subplot(2, 1, 2);
            xlim(x_limits);

            drawnow limitrate; % 'limitrate' to improve performance
        end
    end
end

% Function definition must be at the end of the script
function data = getData(s)
    %% Receive the Data
    try
        datastring = readline(s); % Read a full line from the serial buffer
        disp(datastring); % Debugging: print the raw string received

        % Split the line by commas
        tokens = split(datastring, ',');

        % Convert the tokens to numerical values
        if length(tokens) == 6
            data = str2double(tokens(1:6)); % Extract all 6 values for accelerometer and gyroscope data
        else
            data = []; % If the expected number of values is not present, return empty
        end
    catch
        warning('Failed to read data from serial port');
        data = []; % Return empty in case of error
    end
end
