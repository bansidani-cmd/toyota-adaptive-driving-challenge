function [targetAngle, method] = steering_angle(headAngles, prevTargets, method)
%STEERING_ANGLE Determine the target steering angle from a window of head-angle data.
%
%   [targetAngle, method] = steering_angle(headAngles, prevTargets, method)
%
%   Inputs:
%     headAngles  - vector of head-angle samples for the current window (degrees)
%     prevTargets - vector of the last two target angles, most recent last
%                   (empty or fewer than 2 elements if not yet available)
%     method      - aggregation method: 'mean' | 'derivative' | 'endpoint'
%
%   Output:
%     targetAngle - target wheel angle, snapped to nearest 15 degree increment,
%                   range [0, 180]
%     method      - echoes the method used (for logging)

    if nargin < 3
        method = 'mean';
    end

    validAngles = 0:15:180;

    switch method
        case 'mean'
            rawAngle = mean(headAngles);

        case 'derivative'
            % Weight samples by the magnitude of the local rate of change,
            % so fast head movements influence the estimate more than
            % slow drift.
            if numel(headAngles) < 2
                rawAngle = mean(headAngles);
            else
                d = abs(diff(headAngles));
                weights = [d(1), d]; % pad to match length
                weights = weights + eps; % avoid all-zero weights
                rawAngle = sum(headAngles .* weights) / sum(weights);
            end

        case 'endpoint'
            % Bias toward the final samples in the window (most recent
            % intent), using a linearly increasing weight.
            n = numel(headAngles);
            weights = linspace(1, 3, n);
            rawAngle = sum(headAngles .* weights) / sum(weights);

        otherwise
            error('Unknown method: %s', method);
    end

    snapped = interp1(validAngles, validAngles, rawAngle, 'nearest', 'extrap');
    snapped = min(max(snapped, 0), 180);

    targetAngle = apply_trend_correction(snapped, rawAngle, prevTargets, validAngles);
end

function correctedAngle = apply_trend_correction(snappedAngle, rawAngle, prevTargets, validAngles)
%APPLY_TREND_CORRECTION Adjust the naively-snapped angle using recent trend.
%
%   If the last two target angles show a consistent step direction and
%   size, and the raw (unsnapped) angle this window continues that trend
%   but falls short of a full 15 degree step, continue the trend instead
%   of truncating early. This reduces perceived lag/oscillation in the
%   steering response.

    correctedAngle = snappedAngle;

    if numel(prevTargets) < 2
        return; % not enough history to detect a trend
    end

    step = prevTargets(end) - prevTargets(end-1);
    if step == 0
        return; % no trend to continue
    end

    expectedNext = prevTargets(end) + step;
    if ~ismember(expectedNext, validAngles)
        return; % trend would exceed valid range
    end

    % Does the raw angle support continuing the trend, even partially?
    partialProgress = (rawAngle - prevTargets(end)) * sign(step);
    if partialProgress > 0 && abs(rawAngle - snappedAngle) <= 7.5
        % Raw estimate is moving in the trend direction; continue the step
        % rather than rounding back down/up to the current value.
        if snappedAngle == prevTargets(end)
            correctedAngle = expectedNext;
        end
    end
end
