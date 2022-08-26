%% Default input
DefaultAddress = '10.0.0.33';
DefaultPort = 4000;
MaxSize = 1000;
RunInfity = true;

%% user config
% adress = input('Enter IP address: ');
% if isempty(adress)
%     disp("no address is entered. Using default address: "+ DefaultAddress);
    adress = DefaultAddress;
% end
% 
% port = input('Enter Port number: ');
% if isempty(port)
%     disp("no port is entered. Using default port: "+ num2str(DefaultPort));
    port = DefaultPort;
% else
%     port = str2num(port);
% end
%% read Imu specs
specs;

%% store data
if RunInfity
    MaxSize = 100000;
end
t = zeros(MaxSize, 1);
accel = zeros(MaxSize, 3);
gyro = zeros(MaxSize, 3);
mag = zeros(MaxSize, 3);
of = zeros(MaxSize, 1);

%% plot config
figure;
subplot(2,2,1); hold on;
subtitle('Sampling time');
grid on; xlabel('time (s)'); ylabel('s')

subplot(2,2,2); hold on;
subtitle('Acceleration');
grid on; xlabel('time (s)'); ylabel('m/s^2')

subplot(2,2,3); hold on;
subtitle('Gyroscope');
grid on; xlabel('time (s)'); ylabel('rad/s')

subplot(2,2,4); hold on;
subtitle('Magnetometer (utesla)');
grid on; axis square; xlabel('x'); ylabel('y'); zlabel('z'); view(37.5, 30)
%% create TCP
client = tcpclient(adress, port, "ConnectTimeout",20);

%% get dat and plot it
counter = 0;
last_draw = 1;
while (true)
    % check id data avialable
    num = client.NumBytesAvailable();
    if(num == 0)
        pause(0.1);
        continue;
    end
    data = client.read(num, "string");

    % split all possible packet(w), last is empty
    splitted = data.split(";");

    % decode each packet and store its value
    len_packets = length(splitted) - 1;
    for i = 1 : len_packets
        imu_string = splitted(i);
        imu_data = str2num(imu_string);
        % extract
        if length(imu_data) == 11
            counter = counter + 1;
            t(counter) = imu_data(1)/1000;
            accel(counter, :) = imu_data(2:4) / acc_sen;
            gyro(counter, :) = imu_data(5:7) / gyro_sen;
            mag(counter, :) = imu_data(8:10) / mag_sen;
            of(counter) = imu_data(11);
        end
    end

    w = last_draw:counter;

    % calculate time difference
    dt = w * 0;
    if last_draw > 1 % only if it is not the first draw
        dt = t(w) - t(w-1);
    end

    subplot(2,2,1)
    plot(t(w), dt);

    subplot(2,2,2)
    plot(t(w), accel(w, 1), 'r', t(w), accel(w, 2), 'g', t(w), accel(w, 3), 'b');

    subplot(2,2,3)
    plot(t(w), gyro(w, 1), 'r', t(w), gyro(w, 2), 'g', t(w), gyro(w, 3), 'b');

    subplot(2,2,4)
    plot3(mag(w, 1), mag(w, 2), mag(w, 3), "*");
    pause(0.1)

    % update last draw element
    last_draw = counter;

    if RunInfity && counter == MaxSize
        disp("Maximum array size reached")
        break;
    end
end