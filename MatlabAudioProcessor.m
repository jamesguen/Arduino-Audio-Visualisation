%% Setup
clear; clc; close all;
% Force close any existing serial connections before opening a new one

delete(serialportfind);
clear arduinoObj;

 Setup Serial Communication
arduinoPort = "COM9";  % Change if necessary
baudRate = 115200;
arduinoObj = serialport(arduinoPort, baudRate);

%% Audio To Inputs

% Get audio file
Fl = "C:\Users\james\Documents\MATLAB\VikingHAUWSE3.wav";
[Au, fs] = audioread(Fl); % Read Audio and Sample Rate
Au_signal = mean(Au,2);  % Convert to Mono

% Create Audio Player
player = audioplayer(Au, fs);

% Frequency Band Definitions
HPF = [10, 70, 250, 1000, 3000, 6000]; % High-pass frequencies
LPF = [70, 250, 1000, 3000, 6000, fs/2-1]; % Low-pass frequencies

window = round(0.1 * fs); % window width

if length(HPF) == length(LPF)
    bandNum = length(HPF);
else
    error("Error: HPF and LPF arrays must be of equal length. Script terminated.");
end

% Preallocate Band Levels
bandLevels = zeros(bandNum, size(Au,1));

% Apply Bandpass Filtering with Gaussian Smoothing
for i = 1:bandNum
    % Design Bandpass Filter
    BPF = designfilt('bandpassiir', 'FilterOrder', 20, ...
        'HalfPowerFrequency1', HPF(i), 'HalfPowerFrequency2', LPF(i), 'SampleRate', fs);
    
    audioFilt = filtfilt(BPF, Au_signal); % Apply Bandpass Filtering
    bandLevels(i, :) = smoothdata(abs(audioFilt(:)), 'gaussian', window); % Apply Gaussian Smoothing
end

% Downsample before normalization
downsampleFactor = 200; % Downsampling amount
bandLevels = bandLevels(:, 1:downsampleFactor:end); % Downsample
fs_new = fs / downsampleFactor; % Update sample rate

% Normalize band levels dynamically using percentiles
windowSize = round(5 * fs_new); % 5-second normalization window
p5 = movmedian(bandLevels, windowSize, 2);  % Lower bound (5th percentile)
p95 = movmax(bandLevels, windowSize, 2); % Upper bound (95th percentile)

% Normalize within the adaptive range
bandLevels = (bandLevels - p5) ./ (p95 - p5 + eps);
limiterMax = 255; % Maximum output value

% Define Soft Clipping Function
gain=1;
smoothClip = @(x, inpMax, outMax) outMax * clip(1.5 * (inpMax^2 * x - (1/3) * x.^3) / inpMax^3, 0, 1);
bandLevels = smoothClip(gain * bandLevels, 1, 1); % Apply Clipping

% U8 conversion
bandLevels = floor(255*bandLevels);

% Plot Results
colors = {[0.9, 0.1, 0.1], [0.9, 0.5, 0.1], [0.9, 0.9, 0.1], [0.1, 0.9, 0.1], [0.1, 0.5, 0.9], [0.6, 0.1, 0.9]};
figure; hold on;
time = (0:1:length(bandLevels)-1) / fs_new;
for k = 1:bandNum
    plot(time, bandLevels(k,:), 'Color', colors{k}, 'LineWidth', 2);
end
% Generate labels dynamically based on bandNum
units = repmat("Hz", size(LPF));
units(LPF >= 1000) = "kHz";
% Create display frequency arrays with appropriate scaling and significant figures
HPF_display = HPF;
LPF_display = LPF;

% Convert frequencies >= 1000 to kHz
HPF_display(HPF >= 1000) = HPF(HPF >= 1000) / 1000;
LPF_display(LPF >= 1000) = LPF(LPF >= 1000) / 1000;

% Format with appropriate significant figures
HPF_display = floor(HPF_display .* (1 + 999 * (HPF_display >= 1))) ./ (1 + 999 * (HPF_display >= 1));
LPF_display = floor(LPF_display .* (1 + 999 * (LPF_display >= 1))) ./ (1 + 999 * (LPF_display >= 1));

% Assign units
units = repmat("Hz", size(LPF));
units(LPF >= 1000) = "kHz";

% Generate labels dynamically based on bandNum
legendLabels = arrayfun(@(x) sprintf('%.3g-%.3g %s', HPF_display(x), LPF_display(x), units(x)), 1:bandNum, 'UniformOutput', false);
legend(legendLabels);
legend(legendLabels);
xlabel('Time');
ylabel('Brightness Level');
title('Real-Time LED Brightness');
ylim([0 limiterMax]);

disp("Streaming brightness levels to Arduino...");


%% Arduino Communication

% Wait for synchronization
disp("Waiting for Arduino signal...");

while true % Loop Checking for START
    numBytes = arduinoObj.NumBytesAvailable;
    if numBytes > 0 % Look for message
        data = readline(arduinoObj); % Read message
        disp("Received: " + data);

        if strcmp(strtrim(data), "START") % Read START signal
            disp("Arduino Ready! Measuring Audio Start Time...");
            tic;
            play(player);
            pause(1.1);  % Allow playback to start
            
            % Wait until audio actually starts
            while ~isplaying(player)
                pause(0.001);
            end
            disp("Audio has started!");
            
            tic; % Reset timer
            break;
        end
    end
    pause(0.2);
end

pause(0.0001);

%% Stream Brightness Levels to Arduino
sample = 1;
timeLog = zeros(1, length(bandLevels));  % Store loop execution times
pause(0.14);

while sample < length(bandLevels)
    elapsedTime = toc;  % Get time since loop started
    expectedTime = sample / fs_new;  % Expected playback time

    % Adjust delay dynamically
    delayTime = expectedTime - elapsedTime;
    if delayTime > 0
        pause(delayTime);
    end

    % Send All Band Levels as One Message
    bandDataString = strjoin(string(bandLevels(:, sample))', ',');
    writeline(arduinoObj, bandDataString);
    fprintf("Sent: %s\r", bandDataString);

    % Log Execution Time
    timeLog(sample) = toc;
    sample = sample + 1;
end

disp("Finished sending brightness values!");

%% Debugging: Plot Execution Time Log
figure;
plot(timeLog, 'b', 'LineWidth', 2);
xlabel('Sample Index');
ylabel('Elapsed Time (s)');
title('Execution Time per Sample');
grid on;
