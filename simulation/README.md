# Spooler tension model (MATLAB / Simulink)

Models how the spooler holds filament tension while it starts, runs and stops.

## Files

| File | What it does |
| --- | --- |
| `spooler_params.m` | All parameters in one struct `P`. Values marked *(estimate)* need measuring. |
| `build_spooler_model.m` | Builds `spooler_tension.slx` from scratch. Needs Simulink. |
| `run_spooler.m` | Builds the model, runs a full and a nearly empty source spool, and plots both. **Start here.** |
| `spooler_sim_ode.m` | The same model as a plain MATLAB script, with no Simulink needed. Use it to cross-check. |
| `sim_check.py`, `sim_check.png` | Python version used to check the equations, and its plot. |

## How the model works

- **Winding motor = speed source.** A stepper follows its commanded speed, so the receiving side follows the speed profile: ramp up, run at 0.5 m/s, ramp down.
- **Source spool = passive.** Filament tension pulls it round and the brake resists. Its inertia and radius fall as it empties: r = sqrt(r_core^2 + L d^2 / (4 w k)).
- **Dancer.** The filament loop under the dancer pulley is 2x long. It grows when the source pays out faster than the receiver takes up. The spring balances 2T, so T = k (x_f - x) / 2.
- **Controller.** A PID acts on the dancer position and sets the brake force, plus a feedforward F = T_set r_source / (mu r_drum). The result is clamped to 0–54 N, then passed through a servo lag.

## Results (Python and MATLAB-script versions agree)

- Target tension 5.0 N. While running it stays within 5.02–5.03 N, for both a full spool (r = 95 mm) and a nearly empty one (r = 62 mm).
- During the ramps it moves by about ±0.15 N. The dancer stays within 14.6–17.1 mm of its 30 mm travel.
- Brake force runs at 15–24 N, and reaches up to 48 N when stopping. That fits inside the 54 N the lever and servo can give.
- Peak winding motor torque is about 0.09 N m at 5 N tension.

## Things to measure, then update `spooler_params.m`

mu (pad on drum), J_shell, the dancer spring k and damping c, the servo time constant, and k_pack.
