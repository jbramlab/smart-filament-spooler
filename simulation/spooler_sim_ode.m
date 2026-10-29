% spooler_sim_ode.m  -  the same tension model as the Simulink file, as a plain MATLAB script.
% Use it to check the Simulink results, or when Simulink is not available.
% Fixed-step simulation (dt = 0.2 ms) of: source spool + brake + dancer + PID brake controller.

spooler_params;
dt = 2e-4;

cases = {P.L_full, 'full source spool'; 0.03*P.L_full, 'nearly empty source'};
figure('Name', 'Spooler tension model (script version)');
for c = 1:size(cases, 1)
    L_start = cases{c, 1};
    n = round(P.T_end/dt);
    log = zeros(n, 6);                     % t, v_ref, vs, T, x, Fb
    ws = 0; x = P.x_set; Ls = 0; Lr = 0; I = 0;
    rs0 = sqrt(P.rc^2 + L_start*P.d^2/(4*P.w*P.k_pack));
    Fb = P.T_set*rs0/(P.mu*P.r_d);
    tau_peak = 0;
    for i = 1:n
        t = (i - 1)*dt;
        % speed profile
        if t < P.t_ramp
            v = P.v_run*t/P.t_ramp;
        elseif t < P.t_stop
            v = P.v_run;
        elseif t < P.t_stop + P.t_ramp
            v = P.v_run*(1 - (t - P.t_stop)/P.t_ramp);
        else
            v = 0;
        end
        % plant
        Lsrc = max(L_start - Ls, 0);
        rs = sqrt(P.rc^2 + Lsrc*P.d^2/(4*P.w*P.k_pack));
        rr = sqrt(P.rc^2 + Lr*P.d^2/(4*P.w*P.k_pack));
        Js = P.J_shell + 0.5*P.lam*Lsrc*(P.rc^2 + rs^2);
        vs = ws*rs;
        xd = (vs - v)/2;
        T  = max(0, (P.k_sp*(P.xf - x) - P.c_sp*xd)/2);
        % controller: dancer above the set point = loop too long = tension low = more brake
        e = x - P.x_set;
        I = I + e*dt;
        Fff = P.T_set*rs/(P.mu*P.r_d);
        Fcmd = Fff + P.Kp*e + P.Ki*I + P.Kd*xd;
        if Fcmd > P.Fmax || Fcmd < 0           % clamp and stop the integrator winding up
            Fcmd = min(max(Fcmd, 0), P.Fmax);
            I = I - e*dt;
        end
        Fb = Fb + (Fcmd - Fb)/P.tau_srv*dt;   % servo lag
        tb = P.mu*Fb*P.r_d*tanh(ws/P.eps_w);
        wsd = (T*rs - tb - P.b_s*ws)/Js;
        tau_peak = max(tau_peak, T*rr/(P.N*P.eta));
        log(i, :) = [t, v, vs, T, x, Fb];
        % integrate
        ws = ws + wsd*dt;
        x  = min(max(x + xd*dt, 0), P.xmax);
        Ls = Ls + vs*dt;
        Lr = Lr + v*dt;
    end
    m = log(:, 1) > 5 & log(:, 1) < 21;
    fprintf('%s: tension while running %.2f to %.2f N, dancer %.1f to %.1f mm, brake %.1f to %.1f N, peak motor torque %.3f N m\n', ...
        cases{c, 2}, min(log(m, 4)), max(log(m, 4)), 1000*min(log(:, 5)), 1000*max(log(:, 5)), ...
        min(log(:, 6)), max(log(:, 6)), tau_peak);
    subplot(4, 1, 1); hold on; plot(log(:, 1), log(:, 2), log(:, 1), log(:, 3), '--'); ylabel('line speed (m/s)'); grid on;
    subplot(4, 1, 2); hold on; plot(log(:, 1), log(:, 4)); ylabel('tension (N)'); grid on;
    subplot(4, 1, 3); hold on; plot(log(:, 1), 1000*log(:, 5)); ylabel('dancer (mm)'); grid on;
    subplot(4, 1, 4); hold on; plot(log(:, 1), log(:, 6)); ylabel('brake force (N)'); grid on;
end
xlabel('time (s)');
