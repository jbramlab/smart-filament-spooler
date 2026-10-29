% run_spooler.m  -  builds and runs the spooler tension model, then plots it.
% Two runs: full source spool, then a nearly empty one (3% left).

clear; clc;
spooler_params;
build_spooler_model;

cases = {P.L_full, 'full source spool'; 0.03*P.L_full, 'nearly empty source'};
figure('Name', 'Spooler tension model', 'Position', [100 100 900 900]);
for c = 1:size(cases, 1)
    P.L_start = cases{c, 1};
    rs0   = sqrt(P.rc^2 + P.L_start*P.d^2/(4*P.w*P.k_pack));
    P.Fb0 = P.T_set*rs0/(P.mu*P.r_d);
    out = sim('spooler_tension');
    T  = out.get('T');  x = out.get('x');  Fb = out.get('Fb');  v = out.get('v');  tm = out.get('tau_m');

    run_mask = T.time > 5 & T.time < 21;
    fprintf('%s: tension while running %.2f to %.2f N, dancer %.1f to %.1f mm, peak motor torque %.3f N m\n', ...
        cases{c, 2}, min(T.signals.values(run_mask)), max(T.signals.values(run_mask)), ...
        1000*min(x.signals.values), 1000*max(x.signals.values), max(abs(tm.signals.values)));

    subplot(4, 1, 1); hold on; plot(v.time, v.signals.values(:, 1), v.time, v.signals.values(:, 2), '--');
    ylabel('line speed (m/s)'); grid on;
    subplot(4, 1, 2); hold on; plot(T.time, T.signals.values); ylabel('tension (N)'); grid on;
    subplot(4, 1, 3); hold on; plot(x.time, 1000*x.signals.values); ylabel('dancer (mm)'); grid on;
    subplot(4, 1, 4); hold on; plot(Fb.time, Fb.signals.values); ylabel('brake force (N)'); grid on;
end
xlabel('time (s)');
subplot(4, 1, 2); yline(P.T_set, ':k');
legend(subplot(4, 1, 2), cases(:, 2));
