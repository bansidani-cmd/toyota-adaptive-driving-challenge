# Adaptive Steering Angle Determination — Toyota Adaptive Driving Challenge

A MATLAB/Simulink algorithm that converts a continuous stream of head-angle data into a discrete steering target angle for an adaptive driving interface, developed for the Toyota Adaptive Driving Challenge.

## Problem

**Input:** a continuous sequence of head-angle readings, sampled over a rolling time window (this implementation uses 1-second windows).

**Desired output:** a target wheel angle, snapped to 15° increments across the range 0°–180°, updated once per window.

## Approach

1. **Aggregate the window**: evaluate multiple candidate averaging strategies over the most recent window of head-angle samples:
   - Simple mean of the full window
   - Derivative-weighted mean (weights recent, fast-changing samples more heavily)
   - Endpoint-weighted average (biases toward the most recent samples in the window)
2. **Snap to 15° increments**: round the aggregated angle to the nearest valid target.
3. **Trend-aware correction**:  rather than rounding each window in isolation, the algorithm considers the trend of the last two target angles. If the trend indicates a consistent direction of movement (e.g. two consecutive +15° steps) and the current raw calculation falls short of continuing that trend (e.g. +7° instead of +15°), the algorithm can choose to continue the trend rather than truncate it early, reducing oscillation and lag in the output.

## Files

- `steering_angle.m`: core aggregation + snapping + trend-correction logic
- `simulate_demo.m`: generates synthetic head-angle input and demonstrates the algorithm end-to-end

## Notes

This is a from-scratch reimplementation for portfolio purposes, using synthetic demo data rather than the original competition dataset (which isn't mine to publish).
