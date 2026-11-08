function animate_quad(out, P, speed)
% ANIMATE_QUAD  Simple 3-D animation of the logged flight.
if nargin < 3, speed = 2; end          % playback speed factor
t   = out.pos.Time;
pos = squeeze(out.pos.Data); if size(pos,1) ~= numel(t), pos = pos.'; end
eul = squeeze(out.eul.Data); if size(eul,1) ~= numel(t), eul = eul.'; end
eul = interp1(out.eul.Time, eul, t);

fig = figure('Name', 'Quadcopter animation'); ax = axes(fig); hold(ax, 'on'); grid(ax, 'on');
axis(ax, 'equal'); view(ax, 35, 25);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
lim = [min(pos) - 0.5; max(pos) + 0.5];
axis(ax, [lim(:,1)' lim(:,2)' max(lim(1,3),0) lim(2,3)+0.3]);
plot3(ax, pos(:,1), pos(:,2), pos(:,3), ':', 'Color', [.6 .6 .6]);
m  = [P.motor_xy zeros(4,1)]';              % motor positions in body frame
a1 = plot3(ax, 0, 0, 0, 'r-', 'LineWidth', 3);   % front-right / rear-left arm
a2 = plot3(ax, 0, 0, 0, 'b-', 'LineWidth', 3);   % front-left / rear-right arm
hd = plot3(ax, 0, 0, 0, 'k-', 'LineWidth', 2);   % heading line
ttl = title(ax, '');

tic; k = 1;
while k <= numel(t) && isvalid(fig)
    R  = eul2R(eul(k,:));
    pw = pos(k,:)' + R*m;
    hx = pos(k,:)' + R*[P.L; 0; 0];
    set(a1, 'XData', pw(1,[1 2]), 'YData', pw(2,[1 2]), 'ZData', pw(3,[1 2]));
    set(a2, 'XData', pw(1,[3 4]), 'YData', pw(2,[3 4]), 'ZData', pw(3,[3 4]));
    set(hd, 'XData', [pos(k,1) hx(1)], 'YData', [pos(k,2) hx(2)], 'ZData', [pos(k,3) hx(3)]);
    set(ttl, 'String', sprintf('t = %.1f s', t(k)));
    drawnow limitrate
    k = find(t >= toc*speed, 1);              % real-time playback
    if isempty(k), break; end
end
end

function R = eul2R(e)
cph = cos(e(1)); sph = sin(e(1)); cth = cos(e(2)); sth = sin(e(2));
cps = cos(e(3)); sps = sin(e(3));
R = [cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph;
     sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph;
     -sth,    cth*sph,               cth*cph];
end
