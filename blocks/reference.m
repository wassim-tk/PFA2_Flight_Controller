function [ref, pos_ref] = reference(t)
% Mission profile.  ref = [x; y; z; psi; phi; theta]
% (phi/theta are only used when P.ctl.mode = 0, angle mode)
x = 0; y = 0; z = 0; psi = 0; phi = 0; theta = 0;
if t >= 1,  z   = min(1.5, 0.75*(t - 1)); end    % take-off ramp to 1.5 m
if t >= 6,  x   = 2;      end                    % move 2 m forward
if t >= 11, y   = 2;      end                    % move 2 m left
if t >= 16, psi = pi/2;   end                    % yaw 90 deg
if t >= 21, x = 0; y = 0; end                    % back above take-off point
if t >= 26, z   = max(0, 1.5 - 0.5*(t - 26)); end % landing ramp
ref     = [x; y; z; psi; phi; theta];
pos_ref = [x; y; z];
end
