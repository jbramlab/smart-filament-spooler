% build_spooler_model.m  -  builds spooler_tension.slx from scratch.
% Run spooler_params first (run_spooler.m does both). Needs Simulink.
%
% What the model contains
%   Speed profile  : ramp up, run at P.v_run, ramp down (winding motor = speed source,
%                    because a stepper follows its commanded speed)
%   Plant          : source spool inertia, brake torque, dancer spring, changing spool radii
%   Controller     : PID on dancer position -> brake force, plus a feedforward
%                    F = T_set * r_source / (mu * r_drum), clamped 0..Fmax
%   Servo          : first-order lag on the brake force
%   Outputs        : scope + To Workspace signals T, x, Fb, v, rs, tau_m

mdl = 'spooler_tension';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);

pos = @(x, y, w, h) [x y x+w y+h];

% ---------------- speed profile ----------------
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', pos(30, 60, 30, 30));
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Speed profile'], ...
          'Position', pos(110, 50, 120, 50));
set_script([mdl '/Speed profile'], sprintf([ ...
 'function v = profile(t, P)\n' ...
 '%% commanded line speed (m/s)\n' ...
 'if t < P.t_ramp\n    v = P.v_run*t/P.t_ramp;\n' ...
 'elseif t < P.t_stop\n    v = P.v_run;\n' ...
 'elseif t < P.t_stop + P.t_ramp\n    v = P.v_run*(1 - (t - P.t_stop)/P.t_ramp);\n' ...
 'else\n    v = 0;\nend\n']), {'P'});
add_line(mdl, 'Clock/1', 'Speed profile/1', 'autorouting', 'on');

% ---------------- plant ----------------
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Plant'], ...
          'Position', pos(420, 160, 170, 220));
set_script([mdl '/Plant'], sprintf([ ...
 'function [wsd, xd, vs, T, rs, rr, Fff, tau_m] = plant(ws, x, Ls, Lr, Fb, v_ref, P)\n' ...
 '%% radii from the length on each spool (volume of filament = volume of the ring it fills)\n' ...
 'Lsrc = max(P.L_start - Ls, 0);\n' ...
 'rs = sqrt(P.rc^2 + Lsrc*P.d^2/(4*P.w*P.k_pack));\n' ...
 'rr = sqrt(P.rc^2 + max(Lr, 0)*P.d^2/(4*P.w*P.k_pack));\n' ...
 '%% source spool inertia falls as it empties\n' ...
 'Js = P.J_shell + 0.5*P.lam*Lsrc*(P.rc^2 + rs^2);\n' ...
 '%% receiving side follows the commanded speed (stepper)\n' ...
 'vr = v_ref;\n' ...
 'vs = ws*rs;\n' ...
 '%% dancer: loop length is 2x, it grows when pay-out beats take-up\n' ...
 'xd = (vs - vr)/2;\n' ...
 '%% tension from the dancer spring (2T balances the spring)\n' ...
 'T = max(0, (P.k_sp*(P.xf - x) - P.c_sp*xd)/2);\n' ...
 '%% brake torque (Coulomb friction, smoothed near zero speed)\n' ...
 'tb = P.mu*Fb*P.r_d*tanh(ws/P.eps_w);\n' ...
 'wsd = (T*rs - tb - P.b_s*ws)/Js;\n' ...
 '%% brake feedforward and winding motor torque\n' ...
 'Fff = P.T_set*rs/(P.mu*P.r_d);\n' ...
 'tau_m = T*rr/(P.N*P.eta);\n']), {'P'});

% states
add_block('simulink/Continuous/Integrator', [mdl '/ws (rad per s)'], 'Position', pos(680, 120, 40, 30), ...
          'InitialCondition', '0');
add_block('simulink/Continuous/Integrator', [mdl '/x dancer (m)'], 'Position', pos(680, 180, 40, 30), ...
          'InitialCondition', 'P.x_set', 'LimitOutput', 'on', ...
          'UpperSaturationLimit', 'P.xmax', 'LowerSaturationLimit', '0');
add_block('simulink/Continuous/Integrator', [mdl '/Ls unwound (m)'], 'Position', pos(680, 240, 40, 30), ...
          'InitialCondition', '0');
add_block('simulink/Continuous/Integrator', [mdl '/Lr wound (m)'], 'Position', pos(680, 300, 40, 30), ...
          'InitialCondition', '0');
add_line(mdl, 'Plant/1', 'ws (rad per s)/1', 'autorouting', 'on');
add_line(mdl, 'Plant/2', 'x dancer (m)/1', 'autorouting', 'on');
add_line(mdl, 'Plant/3', 'Ls unwound (m)/1', 'autorouting', 'on');
add_line(mdl, 'Speed profile/1', 'Lr wound (m)/1', 'autorouting', 'on');
add_line(mdl, 'ws (rad per s)/1', 'Plant/1', 'autorouting', 'on');
add_line(mdl, 'x dancer (m)/1', 'Plant/2', 'autorouting', 'on');
add_line(mdl, 'Ls unwound (m)/1', 'Plant/3', 'autorouting', 'on');
add_line(mdl, 'Lr wound (m)/1', 'Plant/4', 'autorouting', 'on');
add_line(mdl, 'Speed profile/1', 'Plant/6', 'autorouting', 'on');

% ---------------- controller ----------------
add_block('simulink/Sources/Constant', [mdl '/x set'], 'Position', pos(780, 430, 50, 30), 'Value', 'P.x_set');
add_block('simulink/Math Operations/Sum', [mdl '/error'], 'Position', pos(870, 410, 30, 30), 'Inputs', '+-');
add_line(mdl, 'x dancer (m)/1', 'error/1', 'autorouting', 'on');
add_line(mdl, 'x set/1', 'error/2', 'autorouting', 'on');
add_block('simulink/Continuous/PID Controller', [mdl '/Tension PID'], 'Position', pos(940, 405, 70, 40), ...
          'Controller', 'PID', 'P', 'P.Kp', 'I', 'P.Ki', 'D', 'P.Kd', 'N', 'P.Nf', ...
          'LimitOutput', 'on', 'UpperSaturationLimit', 'P.Fmax', 'LowerSaturationLimit', '-P.Fmax', ...
          'AntiWindupMode', 'clamping');
add_line(mdl, 'error/1', 'Tension PID/1', 'autorouting', 'on');
add_block('simulink/Math Operations/Sum', [mdl '/plus feedforward'], 'Position', pos(1060, 410, 30, 30), 'Inputs', '++');
add_line(mdl, 'Tension PID/1', 'plus feedforward/1', 'autorouting', 'on');
add_line(mdl, 'Plant/7', 'plus feedforward/2', 'autorouting', 'on');
add_block('simulink/Discontinuities/Saturation', [mdl '/0 to Fmax'], 'Position', pos(1120, 410, 40, 30), ...
          'UpperLimit', 'P.Fmax', 'LowerLimit', '0');
add_line(mdl, 'plus feedforward/1', '0 to Fmax/1', 'autorouting', 'on');

% servo: first-order lag built from an integrator so it can start at P.Fb0
add_block('simulink/Math Operations/Sum', [mdl '/servo error'], 'Position', pos(1190, 410, 30, 30), 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [mdl '/1 over tau'], 'Position', pos(1250, 410, 50, 30), 'Gain', '1/P.tau_srv');
add_block('simulink/Continuous/Integrator', [mdl '/Fb brake (N)'], 'Position', pos(1330, 410, 40, 30), ...
          'InitialCondition', 'P.Fb0');
add_line(mdl, '0 to Fmax/1', 'servo error/1', 'autorouting', 'on');
add_line(mdl, 'servo error/1', '1 over tau/1', 'autorouting', 'on');
add_line(mdl, '1 over tau/1', 'Fb brake (N)/1', 'autorouting', 'on');
add_line(mdl, 'Fb brake (N)/1', 'servo error/2', 'autorouting', 'on');
add_line(mdl, 'Fb brake (N)/1', 'Plant/5', 'autorouting', 'on');

% ---------------- outputs ----------------
outs = {'T', 4; 'rs', 5; 'tau_m', 8};
for k = 1:size(outs, 1)
    nm = outs{k, 1};
    add_block('simulink/Sinks/To Workspace', [mdl '/' nm], 'Position', pos(760, 480 + 50*k, 60, 26), ...
              'VariableName', nm, 'SaveFormat', 'Structure With Time');
    add_line(mdl, sprintf('Plant/%d', outs{k, 2}), [nm '/1'], 'autorouting', 'on');
end
add_block('simulink/Sinks/To Workspace', [mdl '/x'], 'Position', pos(760, 700, 60, 26), ...
          'VariableName', 'x', 'SaveFormat', 'Structure With Time');
add_line(mdl, 'x dancer (m)/1', 'x/1', 'autorouting', 'on');
add_block('simulink/Sinks/To Workspace', [mdl '/Fb'], 'Position', pos(1420, 470, 60, 26), ...
          'VariableName', 'Fb', 'SaveFormat', 'Structure With Time');
add_line(mdl, 'Fb brake (N)/1', 'Fb/1', 'autorouting', 'on');
add_block('simulink/Signal Routing/Mux', [mdl '/speeds'], 'Position', pos(760, 760, 8, 50), 'Inputs', '2');
add_line(mdl, 'Speed profile/1', 'speeds/1', 'autorouting', 'on');
add_line(mdl, 'Plant/3', 'speeds/2', 'autorouting', 'on');
add_block('simulink/Sinks/To Workspace', [mdl '/v'], 'Position', pos(820, 770, 60, 26), ...
          'VariableName', 'v', 'SaveFormat', 'Structure With Time');
add_line(mdl, 'speeds/1', 'v/1', 'autorouting', 'on');

add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', pos(1450, 160, 50, 120), 'NumInputPorts', '3');
add_line(mdl, 'Plant/4', 'Scope/1', 'autorouting', 'on');
add_line(mdl, 'x dancer (m)/1', 'Scope/2', 'autorouting', 'on');
add_line(mdl, 'Fb brake (N)/1', 'Scope/3', 'autorouting', 'on');

set_param(mdl, 'StopTime', 'P.T_end', 'Solver', 'ode23t', 'MaxStep', '1e-3', 'RelTol', '1e-5');
save_system(mdl);
fprintf('Built %s.slx\n', mdl);

% ---------------- helper ----------------
function set_script(blk, code, params)
    % puts code in a MATLAB Function block and marks the named inputs as parameters
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
    ch.Script = code;
    for i = 1:numel(params)
        dd = ch.find('-isa', 'Stateflow.Data', 'Name', params{i});
        dd.Scope = 'Parameter';
    end
end
