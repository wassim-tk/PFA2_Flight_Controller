function build_quad_model()
% BUILD_QUAD_MODEL  Creates quad_sim.slx (quadcopter + cascaded PID controller).
% The code of each MATLAB Function block is read from the files in ./blocks,
% so edit those files and re-run this script to rebuild the model.
%
%   Clock -> Reference -> ZOH -> Controller -> Mixer -> Plant (6DOF) -> Integrator
%                                   ^                                     |
%                                   +---- ZOH <- Sensors(+noise) <- StateOut

here = fileparts(mfilename('fullpath'));
bdir = fullfile(here, 'blocks');
addpath(here, bdir);
evalin('base', 'quad_params;');

mdl = 'quad_sim';
if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(here, [mdl '.slx']);
if exist(f, 'file'), delete(f); end
new_system(mdl);
open_system(mdl);
set_param(mdl, 'Solver', 'ode45', 'StopTime', 'P.Tsim', 'MaxStep', '1e-3', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'InitFcn', 'if ~exist(''P'',''var''), quad_params; end');

%% ---------------- blocks
b = @(name) [mdl '/' name];

add_block('simulink/Sources/Clock', b('Clock'), 'Position', [20 100 50 130]);
add_mfb(mdl, b('Reference'),  fullfile(bdir,'reference.m'),     [90  80 200 150]);
add_block('simulink/Discrete/Zero-Order Hold', b('ZOH_ref'), 'SampleTime','P.Ts', ...
          'Position', [250 95 285 125]);
add_mfb(mdl, b('Controller'), fullfile(bdir,'controller.m'),    [340 80 470 190]);
add_mfb(mdl, b('Mixer'),      fullfile(bdir,'mixer.m'),         [520 95 620 135]);
add_mfb(mdl, b('Plant'),      fullfile(bdir,'quad_dynamics.m'), [680 80 800 150]);
add_block('simulink/Continuous/Integrator', b('Integrator'), ...
          'InitialCondition', 'P.x0', 'Position', [850 100 880 130]);
add_mfb(mdl, b('StateOut'),   fullfile(bdir,'state_out.m'),     [930 80 1040 200]);
add_block('simulink/Sources/Random Number', b('Noise'), ...
          'Mean', 'zeros(1,12)', 'Variance', 'P.sens.enable*P.sens.std.^2', ...
          'Seed', '1:12', 'SampleTime', 'P.Ts', 'Position', [930 260 970 290]);
add_mfb(mdl, b('Sensors'),    fullfile(bdir,'sensors.m'),       [1010 240 1100 300]);
add_block('simulink/Discrete/Zero-Order Hold', b('ZOH_meas'), 'SampleTime','P.Ts', ...
          'Position', [1140 255 1175 285]);

% scopes
add_block('simulink/Sinks/Scope', b('Scope_position'), 'NumInputPorts','2', 'Position',[1150 60 1190 110]);
add_block('simulink/Sinks/Scope', b('Scope_attitude'), 'NumInputPorts','2', 'Position',[1150 130 1190 180]);
add_block('simulink/Sinks/Scope', b('Scope_motors'),   'NumInputPorts','1', 'Position',[1150 200 1190 230]);

% logging (out.<name> after sim)
logs = {'pos','pos_ref','eul','att_sp','w_mot','u_ctrl'};
for k = 1:numel(logs)
    add_block('simulink/Sinks/To Workspace', b(['log_' logs{k}]), ...
        'VariableName', logs{k}, 'SaveFormat', 'Timeseries', ...
        'Position', [1250 40+50*k 1320 60+50*k]);
end

%% ---------------- wiring
L = @(src, dst) add_line(mdl, src, dst, 'autorouting', 'on');
L('Clock/1',      'Reference/1');
L('Reference/1',  'ZOH_ref/1');
L('ZOH_ref/1',    'Controller/1');
L('Reference/2',  'Scope_position/2');
L('Reference/2',  'log_pos_ref/1');
L('Controller/1', 'Mixer/1');
L('Controller/1', 'log_u_ctrl/1');
L('Controller/2', 'Scope_attitude/2');
L('Controller/2', 'log_att_sp/1');
L('Mixer/1',      'Plant/2');
L('Plant/1',      'Integrator/1');
L('Integrator/1', 'Plant/1');
L('Integrator/1', 'StateOut/1');
L('StateOut/1',   'Sensors/1');
L('Noise/1',      'Sensors/2');
L('Sensors/1',    'ZOH_meas/1');
L('ZOH_meas/1',   'Controller/2');
L('StateOut/2',   'Scope_position/1');
L('StateOut/2',   'log_pos/1');
L('StateOut/3',   'Scope_attitude/1');
L('StateOut/3',   'log_eul/1');
L('StateOut/4',   'Scope_motors/1');
L('StateOut/4',   'log_w_mot/1');

try
    Simulink.BlockDiagram.arrangeSystem(mdl);   % R2018a+
catch
end
save_system(mdl, f);
fprintf('Model saved: %s\n', f);
end

%% ------------------------------------------------------------------------
function add_mfb(mdl, blk, file, pos)
% Adds a MATLAB Function block whose code comes from FILE.
% An argument called P becomes a 'Parameter' (read from the base workspace).
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
rt = sfroot;
m  = rt.find('-isa', 'Simulink.BlockDiagram', 'Name', mdl);
ch = m.find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = fileread(file);
pd = ch.find('-isa', 'Stateflow.Data', 'Name', 'P');
if ~isempty(pd)
    pd.Scope = 'Parameter';
end
end
