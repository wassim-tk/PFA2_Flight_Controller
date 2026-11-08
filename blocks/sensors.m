function y_meas = sensors(y, n)
% Measured state = true state + noise.  Replace with an IMU/baro/GPS model
% and a state estimator (complementary / Kalman filter) later.
y_meas = y + n(:);
end
