function [y, pos, eul, w] = state_out(x)
% Splits the state vector into the signals that leave the plant.
y   = x(1:12);     % [pos; vel; eul; omega] -> goes to the sensors/controller
pos = x(1:3);
eul = x(7:9);
w   = x(13:16);    % motor speeds [rad/s]
end
