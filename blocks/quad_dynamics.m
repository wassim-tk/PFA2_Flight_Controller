function dx = quad_dynamics(x, w_cmd, P)
% Quadcopter 6-DOF rigid body + first-order motors (Newton-Euler).
% x = [pos(3); vel(3); eul(3)=[phi theta psi]; omega(3)=[p q r]; w(4)]
% w_cmd = commanded motor speeds [rad/s]
vel   = x(4:6);
eul   = x(7:9);
omega = x(10:12);
w     = x(13:16);

% --- motors
w_c = min(max(w_cmd, 0), P.w_max);
dw  = (w_c - w) / P.tau_m;
F   = P.kf * w.^2;               % thrust of each motor
u   = P.A * F;                   % [T; tau_x; tau_y; tau_z]
T   = u(1);
tau = u(2:4);

% --- rotation body -> inertial (ZYX)
cph = cos(eul(1)); sph = sin(eul(1));
cth = cos(eul(2)); sth = sin(eul(2));
cps = cos(eul(3)); sps = sin(eul(3));
R = [cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph;
     sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph;
     -sth,    cth*sph,               cth*cph];

% --- translational dynamics
dpos = vel;
acc  = [0; 0; -P.g] + R*[0; 0; T]/P.m - (P.kd/P.m)*vel;

% --- rotational dynamics
Wm = [1, sph*sth/cth, cph*sth/cth;
      0, cph,         -sph;
      0, sph/cth,     cph/cth];
deul   = Wm * omega;
domega = P.J \ (tau - cross(omega, P.J*omega));

% --- simple ground contact (z = 0 plane)
if x(3) <= 0 && acc(3) <= 0
    dpos   = [0; 0; 0];
    acc    = -vel / 0.02;
    deul   = [0; 0; 0];
    domega = -omega / 0.02;
end

dx = [dpos; acc; deul; domega; dw];
end
