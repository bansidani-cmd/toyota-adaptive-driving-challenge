%SIMULATE_DEMO Demonstrates steering_angle.m on synthetic head-angle data.
%
%   Generates a synthetic head-angle trace simulating a driver gradually
%   turning their head from ~10 degrees to ~100 degrees over time, with
%   added sensor noise, then runs it through the steering angle algorithm
%   in 1-second windows.

clear; clc;

fs = 50;              % samples per second (simulated head-tracker rate)
windowSeconds = 1;     % window length, matches original challenge setup
totalSeconds = 12;
t = 0:1/fs:totalSeconds;

% Synthetic smooth head-angle trend (deg) with noise
trueAngle = 10 + 90 * (1 ./ (1 + exp(-0.6*(t - 6)))); % sigmoid ramp 10 -> 100 deg
noise = 2 * randn(size(t));
headAngleTrace = trueAngle + noise;

samplesPerWindow = fs * windowSeconds;
numWindows = floor(numel(headAngleTrace) / samplesPerWindow);

methods = {'mean', 'derivative', 'endpoint'};
results = struct();

for m = 1:numel(methods)
    method = methods{m};
    targets = [];
    for w = 1:numWindows
        idx = (w-1)*samplesPerWindow + 1 : w*samplesPerWindow;
        window = headAngleTrace(idx);
        prevTargets = targets(max(1, end-1):end);
        [targetAngle, ~] = steering_angle(window, prevTargets, method);
        targets(end+1) = targetAngle; %#ok<SAGROW>
    end
    results.(method) = targets;
    fprintf('%s method target sequence: %s\n', method, mat2str(targets));
end

figure;
hold on;
plot(t, trueAngle, 'k--', 'DisplayName', 'True head angle (deg)');
colors = lines(numel(methods));
for m = 1:numel(methods)
    method = methods{m};
    stepTimes = (1:numWindows) * windowSeconds;
    stairs(stepTimes, results.(method), 'Color', colors(m,:), 'LineWidth', 1.5, ...
        'DisplayName', [method ' -> target angle']);
end
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Location', 'southeast');
title('Steering target angle vs. true head angle, by aggregation method');
grid on;
