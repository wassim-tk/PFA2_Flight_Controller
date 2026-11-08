%% linearize_hover.m  --  linear model around hover (for PID / LQR design)
% Uses the same dynamics file as the Simulink Plant block.
%   x12 = [pos; vel; eul; omega],  u = [T; tau_x; tau_y; tau_z]
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'blocks'));
quad_params;

z0  = 1;                                  % linearize in the air (no ground contact)
x0  = [0;0;z0; zeros(9,1)];
u0  = [P.m*P.g; 0; 0; 0];
w_of_u = @(u) sqrt(max(P.Ainv*u, 0)/P.kf);
f12 = @(x,u) subsref(quad_dynamics([x; w_of_u(u)], w_of_u(u), P), ...
                     struct('type','()','subs',{{1:12}}));

fprintf('|f(x0,u0)| = %.2e  (should be ~0 at hover)\n', norm(f12(x0,u0)));

h = 1e-6;
A = zeros(12); B = zeros(12,4);
for i = 1:12
    e = zeros(12,1); e(i) = h;
    A(:,i) = (f12(x0+e,u0) - f12(x0-e,u0)) / (2*h);
end
for i = 1:4
    e = zeros(4,1); e(i) = h;
    B(:,i) = (f12(x0,u0+e) - f12(x0,u0-e)) / (2*h);
end
A(abs(A) < 1e-9) = 0;  B(abs(B) < 1e-9) = 0;
disp('A ='); disp(A);  disp('B ='); disp(B);

% Decoupled channels you can use to tune each PID separately:
fprintf('roll : phi/tau_x   = 1/(%.4g s^2)\n', P.J(1,1));
fprintf('pitch: theta/tau_y = 1/(%.4g s^2)\n', P.J(2,2));
fprintf('yaw  : psi/tau_z   = 1/(%.4g s^2)\n', P.J(3,3));
fprintf('alt  : z/T         = 1/(%.4g s^2 + %.4g s)\n', P.m, P.kd);

if exist('ss', 'file') == 2 || exist('ss', 'class')
    sys = ss(A, B, eye(12), 0);  %#ok<NASGU>
    fprintf('State-space object "sys" created (Control System Toolbox).\n');
end
save(fullfile(here, 'quad_lin.mat'), 'A', 'B', 'x0', 'u0');
