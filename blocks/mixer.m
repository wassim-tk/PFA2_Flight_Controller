function w_cmd = mixer(u, P)
% Converts [T; tau_x; tau_y; tau_z] into the 4 motor speed commands.
% On the real board this is where you go to PWM / DShot values.
F = P.Ainv * u;              % thrust needed from each motor [N]
F = max(F, 0);               % props can't push down
w_cmd = sqrt(F / P.kf);
w_cmd = min(w_cmd, P.w_max);
end
