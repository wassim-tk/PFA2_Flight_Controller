function [u, att_sp] = controller(ref, y, P)
% Cascaded flight controller, runs at P.Ts (discrete).
%   position -> tilt angles -> body rates -> torques ;  altitude -> thrust
% ref = [x y z psi phi theta],  y = measured [pos; vel; eul; omega]
% u   = [T; tau_x; tau_y; tau_z],  att_sp = [phi_sp; theta_sp; psi_sp]
persistent iz irate omega_prev dfilt zref_prev
if isempty(iz)
    iz = 0; irate = zeros(3,1); omega_prev = zeros(3,1); dfilt = zeros(3,1);
    zref_prev = 0;
end
Ts = P.Ts;  C = P.ctl;
pos = y(1:3); vel = y(4:6); eul = y(7:9); omega = y(10:12);

% ---------- disarmed on the ground: motors off, reset integrators
if ref(3) <= 0 && pos(3) < 0.05
    iz = 0; irate = zeros(3,1); dfilt = zeros(3,1); omega_prev = omega;
    zref_prev = ref(3);
    u = zeros(4,1);
    att_sp = [0; 0; ref(4)];
    return
end

% ---------- outer loop: position -> desired roll/pitch
if C.mode == 1
    e_p = ref(1:2) - pos(1:2);
    a_d = C.pos.Kp(:).*e_p - C.pos.Kd(:).*vel(1:2);
    a_d = min(max(a_d, -C.acc_max), C.acc_max);
    psi = eul(3);
    ax_b =  cos(psi)*a_d(1) + sin(psi)*a_d(2);   % rotate into yaw frame
    ay_b = -sin(psi)*a_d(1) + cos(psi)*a_d(2);
    theta_d = atan2(ax_b, P.g);
    phi_d   = atan2(-ay_b*cos(theta_d), P.g);
else
    phi_d   = ref(5);
    theta_d = ref(6);
end
phi_d   = min(max(phi_d,   -C.tilt_max), C.tilt_max);
theta_d = min(max(theta_d, -C.tilt_max), C.tilt_max);

% ---------- altitude loop -> collective thrust
vz_ref = (ref(3) - zref_prev)/Ts;               % climb-rate feed-forward
vz_ref = min(max(vz_ref, -C.vz_max), C.vz_max);  % (clipped so z steps don't spike)
zref_prev = ref(3);
ez = ref(3) - pos(3);
iz = min(max(iz + C.alt.Ki*ez*Ts, -C.alt.Imax), C.alt.Imax);
T  = P.m*P.g + C.alt.Kp*ez + C.alt.Kd*(vz_ref - vel(3)) + iz;
T  = T / max(cos(eul(1))*cos(eul(2)), 0.5);      % tilt compensation
T  = min(max(T, 0), C.T_max);

% ---------- attitude loop (P) -> rate setpoints
att_sp = [phi_d; theta_d; ref(4)];
e_att  = att_sp - eul;
e_att(3) = atan2(sin(e_att(3)), cos(e_att(3)));  % wrap yaw error
rate_sp = C.att.Kp(:).*e_att;
rate_sp = min(max(rate_sp, -C.rate_max(:)), C.rate_max(:));

% ---------- rate loop (PID, D on measurement, filtered)
e_r   = rate_sp - omega;
irate = min(max(irate + C.rate.Ki(:).*e_r*Ts, -C.rate.Imax(:)), C.rate.Imax(:));
d_raw = (omega - omega_prev)/Ts;
omega_prev = omega;
alpha = Ts/(Ts + 1/(2*pi*C.rate.dfc));
dfilt = dfilt + alpha*(d_raw - dfilt);
tau = C.rate.Kp(:).*e_r + irate - C.rate.Kd(:).*dfilt;

u = [T; tau];
end
