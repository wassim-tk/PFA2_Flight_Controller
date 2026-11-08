%% quad_params.m  --  Parameters for the quadcopter Simulink model (X frame)
% Run this before simulating (the model also runs it on load).
% All units SI. Body frame: x forward, y left, z up. Euler angles ZYX
% (phi = roll about x, theta = pitch about y, psi = yaw about z).
%
% Motor layout (seen from above):          3 (FL, CW)    1 (FR, CCW)
%                                                   \  ^  /
%                                                    \ | /   x forward
%                                                    /   \
%                                                   /     \
%                                          2 (RL, CCW)   4 (RR, CW)
% Replace the numbers below with your own drone's measured values.

P = struct();

%% Simulation
P.Ts   = 1/250;     % flight-controller loop period [s] (250 Hz, realistic for STM32F103)
P.Tsim = 32;        % simulation stop time [s]

%% Environment
P.g = 9.81;         % [m/s^2]

%% Airframe (typical 450 mm frame, 3S, 10" props)
P.m  = 1.2;                         % total mass [kg]
P.L  = 0.225;                       % centre-to-motor distance [m]
P.J  = diag([7.5e-3 7.5e-3 1.3e-2]);% inertia [kg m^2]
P.kd = 0.10;                        % linear aero drag [N/(m/s)]

%% Propulsion (F = kf*w^2, Q = km*w^2, w in rad/s)
P.kf    = 8.5e-6;   % thrust coefficient [N/(rad/s)^2]
P.km    = 1.36e-7;  % drag-torque coefficient [Nm/(rad/s)^2]
P.tau_m = 0.03;     % motor+ESC time constant [s]
P.w_max = 950;      % max prop speed [rad/s] (~9000 rpm)

%% Mixer / allocation:  [T; tau_x; tau_y; tau_z] = A * [F1; F2; F3; F4]
d = P.L/sqrt(2);
c = P.km/P.kf;
P.motor_xy  = [ d -d;  -d  d;   d  d;  -d -d];   % [x y] of motors 1..4
P.motor_dir = [ 1  1  -1  -1];                   % +1 = CCW seen from above
P.A = [ ones(1,4);
        P.motor_xy(:,2)';          % roll  torque = y_i * F_i
       -P.motor_xy(:,1)';          % pitch torque = -x_i * F_i
       -c*P.motor_dir ];           % yaw   torque (reaction of prop drag)
P.Ainv    = inv(P.A);
P.w_hover = sqrt(P.m*P.g/(4*P.kf));

%% Initial state  x = [pos(3); vel(3); eul(3); omega(3); w_motor(4)]
P.x0 = zeros(16,1);          % sitting on the ground, motors stopped

%% Sensors (additive white noise on the measured state, 0 = off)
P.sens.enable = 1;
P.sens.std = [0.02 0.02 0.02, ...        % position [m]
              0.05 0.05 0.05, ...        % velocity [m/s]
              deg2rad([0.5 0.5 1]), ...  % angles   [rad]
              deg2rad([1 1 1])];         % gyro     [rad/s]

%% Controller (cascaded PID, like PX4/Betaflight)
P.ctl.mode = 1;              % 1 = position hold (x,y,z,yaw refs), 0 = angle mode (roll/pitch refs + altitude hold)

% position loop (x,y) -> desired acceleration -> desired tilt
P.ctl.pos.Kp   = [1.0 1.0];  % [1/s^2]
P.ctl.pos.Kd   = [1.8 1.8];  % [1/s]
P.ctl.acc_max  = 4;          % [m/s^2]
P.ctl.tilt_max = deg2rad(25);

% altitude loop -> collective thrust
P.ctl.alt.Kp   = 4.8;        % [N/m]
P.ctl.alt.Kd   = 4.3;        % [N/(m/s)]
P.ctl.alt.Ki   = 1.5;        % [N/(m s)]
P.ctl.alt.Imax = 3;          % [N]
P.ctl.vz_max   = 1.5;        % climb-rate feed-forward limit [m/s]
P.ctl.T_max    = 0.95*4*P.kf*P.w_max^2;

% attitude (angle) loop -> rate setpoint
P.ctl.att.Kp   = [6 6 2];                 % [1/s]
P.ctl.rate_max = deg2rad([200 200 60]);   % [rad/s]

% body-rate loop -> torques
P.ctl.rate.Kp   = [0.15  0.15  0.08];     % [Nm/(rad/s)]
P.ctl.rate.Ki   = [0.05  0.05  0.03];
P.ctl.rate.Kd   = [0.002 0.002 0];
P.ctl.rate.Imax = [0.1 0.1 0.1];          % [Nm]
P.ctl.rate.dfc  = 30;                     % D-term low-pass cutoff [Hz]

clear d c
