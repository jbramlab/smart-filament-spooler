% spooler_params.m  -  parameters for the Smart Filament Spooler tension model
% Run this first (run_spooler.m calls it). Everything is in SI units.
% Values marked (estimate) are first guesses to be replaced by measurements.

P = struct();

% ---- filament and spools ----
P.d       = 1.75e-3;                 % filament diameter (m)
P.rho     = 1240;                    % PLA density (kg/m^3)
P.lam     = P.rho*pi*P.d^2/4;        % mass per metre (kg/m), about 3 g/m
P.w       = 0.061;                   % wound width inside the flanges (m), Bambu 1 kg spool
P.k_pack  = 0.80;                    % packing density of the wound filament (estimate)
P.rc      = 0.061;                   % spool core radius (m), 122 mm core
P.L_full  = 1.0/P.lam;               % filament length on a full 1 kg spool (m), about 335 m
P.L_start = P.L_full;                % length on the source spool at t = 0 (change to test an emptier spool)
P.J_shell = 0.0025;                  % empty source spool + cones + brake drum inertia (kg m^2) (estimate)
P.b_s     = 1e-4;                    % source bearing viscous friction (N m s) (estimate)

% ---- brake: servo, 4:1 lever, TPU pad on the drum ----
P.mu      = 0.5;                     % TPU pad on PLA drum friction (estimate)
P.r_d     = 0.040;                   % brake drum radius (m)
P.Fmax    = 54;                      % maximum pad force (N)
P.tau_srv = 0.08;                    % servo response time constant (s) (estimate)
P.eps_w   = 0.005;                   % smoothing of the friction sign near zero speed (rad/s)

% ---- dancer arm ----
P.k_sp    = 670;                     % dancer spring stiffness at the pulley (N/m) (design value)
P.c_sp    = 5;                       % dancer damping (N s/m) (estimate)
P.xf      = 0.030;                   % pulley position where the spring force is zero (m)
P.xmax    = 0.030;                   % dancer travel (m)
P.x_set   = 0.015;                   % dancer set point, mid travel (m)
P.T_set   = P.k_sp*(P.xf - P.x_set)/2;   % target filament tension (N), 5.0 N

% ---- winding drive ----
P.N       = 68/17;                   % gear ratio
P.eta     = 0.9;                     % gear efficiency (estimate)

% ---- job profile ----
P.v_run   = 0.5;                     % line speed while running (m/s)
P.t_ramp  = 3;                       % ramp up / down time (s)
P.t_stop  = 22;                      % time the ramp down starts (s)
P.T_end   = 30;                      % simulation length (s)

% ---- tension controller (PID on dancer position -> brake force) ----
P.Kp      = 1500;                    % N per m of dancer error
P.Ki      = 6000;                    % N per m s
P.Kd      = 250;                     % N per m/s
P.Nf      = 200;                     % derivative filter (rad/s)

% starting brake force = feedforward for the starting radius
rs0  = sqrt(P.rc^2 + P.L_start*P.d^2/(4*P.w*P.k_pack));
P.Fb0 = P.T_set*rs0/(P.mu*P.r_d);

fprintf('Target tension %.2f N, start radius %.1f mm, starting brake force %.1f N\n', ...
        P.T_set, rs0*1000, P.Fb0);
