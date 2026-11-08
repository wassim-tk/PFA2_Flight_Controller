function plot_results(out)
% PLOT_RESULTS  Plots the signals logged by quad_sim.
[t,  pos]  = tsdata(out.pos);
[tr, pref] = tsdata(out.pos_ref);
[te, eul]  = tsdata(out.eul);
[ta, asp]  = tsdata(out.att_sp);
[tw, w]    = tsdata(out.w_mot);
[tu, u]    = tsdata(out.u_ctrl);

figure('Name', 'Position'); lbl = {'x','y','z'};
for k = 1:3
    subplot(3,1,k); plot(t, pos(:,k), tr, pref(:,k), '--'); grid on
    ylabel([lbl{k} ' [m]']); legend('actual','reference','Location','best');
end
xlabel('t [s]');

figure('Name', 'Attitude'); lbl = {'\phi roll','\theta pitch','\psi yaw'};
for k = 1:3
    subplot(3,1,k); plot(te, rad2deg(eul(:,k)), ta, rad2deg(asp(:,k)), '--'); grid on
    ylabel([lbl{k} ' [deg]']); legend('actual','setpoint','Location','best');
end
xlabel('t [s]');

figure('Name', 'Actuators');
subplot(2,1,1); plot(tw, w*60/(2*pi)); grid on; ylabel('motor speed [rpm]');
legend('M1 FR','M2 RL','M3 FL','M4 RR','Location','best');
subplot(2,1,2); yyaxis left;  plot(tu, u(:,1)); ylabel('thrust T [N]');
yyaxis right; plot(tu, u(:,2:4)); ylabel('torques [Nm]'); grid on; xlabel('t [s]');
legend('T','\tau_x','\tau_y','\tau_z','Location','best');
end

function [t, d] = tsdata(ts)
% Returns time and an N-by-n data matrix whatever the logged signal shape.
t = ts.Time;
d = squeeze(ts.Data);
if size(d, 1) ~= numel(t), d = d.'; end
end
