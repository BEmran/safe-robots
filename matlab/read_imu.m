%% Import data from text file
[file,path] = uigetfile('*.txt');
filename = fullfile(path,file);
if isequal(file,0)
   error('User selected Cancel');
else
   disp(['User selected ', filename]);
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 11);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = [" ", ","];

% Specify column names and types
opts.VariableNames = ["t", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "of"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";

% Specify variable properties
opts = setvaropts(opts, ["t", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "of"], "FillValue", 0);
opts = setvaropts(opts, ["t", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "of"], "ThousandsSeparator", ",");

% Import the data
imu = readtable(filename, opts);

% Clear temporary variables
clear opts

%% Convert to output type
specs()
t = imu.t / mil_to_sec;
accel = [imu.ax, imu.ay, imu.az] / acc_sen;
gyro = [imu.gx, imu.gy, imu.gz] / gyro_sen;
mag = [imu.mx, imu.my, imu.mz] / mag_sen;
of = imu.of;
% claculate sampling time
dt = zeros(length(t), 1);
dt(2:end) = t(2:end) - t(1:end-1);
disp(["Sampling time Average:", sum(dt)/length(dt),", Maximum: ", max(dt), ", Minimum: ", min(dt(2:end))]);

% caculate acceleration norml
a_norm = sqrt(accel(:, 1).^2 + accel(:, 2).^2 + accel(:, 3).^2);

%% plot
figure;
subplot(2,2,1)
plot(t, dt); legend('dt');
subtitle('Sampling time'); grid on; xlabel('time (s)'); ylabel('s')

subplot(2,2,2)
plot(t, accel(:, 1), 'r', t, accel(:, 2), 'g', t, accel(:, 3), 'b', t, a_norm, '-k');
subtitle('Acceleration'); grid on; legend('ax', 'ay', 'az', 'norm'); xlabel('time (s)'); ylabel('m/s^2')

subplot(2,2,3)
plot(t, gyro(:, 1), 'r', t, gyro(:, 2), 'g', t, gyro(:, 3), 'b');
subtitle('Gyroscope'); grid on; legend('gx', 'gy', 'gz'); xlabel('time (s)'); ylabel('rad/s')

subplot(2,2,4)
scatter3(mag(:, 1), mag(:, 2), mag(:, 3));
subtitle('Magnetometer (utesla)'); grid on; axis square; xlabel('x'); ylabel('y'); zlabel('z');
