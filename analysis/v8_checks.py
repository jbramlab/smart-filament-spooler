"""Engineering checks for the v8 design changes (splicer removed; tail clamp,
runout lever, breaker and e-stop added). Assumptions are listed beside each
input so they can be replaced with measured values later."""
import math

T = 5.0            # N, set tension
T_rs = 2.0         # N, restart tension after a splice
mu_tpu = 0.5       # TPU pad on PLA (assumed, to be measured)
mu_pla = 0.3       # PLA on PLA (assumed, to be measured)
d_f = 1.75e-3      # m, filament diameter
A_f = math.pi * d_f**2 / 4
E_pla = 3.5e9      # Pa, PLA Young's modulus (typical)
sig_uts = 50e6     # Pa, PLA filament tensile strength (typical, conservative)
F_break = sig_uts * A_f

out = []
def say(s=''): out.append(s); print(s)

say('1. Tail clamp (splice point)')
N_req = T / (mu_tpu + mu_pla)            # pad on one side, PLA channel on the other
say(f'  Pad force to hold {T:.0f} N (no safety factor): N = T/(mu_TPU + mu_PLA) = {N_req:.1f} N')
say(f'  With a safety factor of 3: {3*N_req:.0f} N')
K, d_s = 0.2, 3e-3                        # nut factor, M3 thread
for tq in (0.05, 0.15):
    F = tq / (K * d_s)
    p = F / (8e-3 * d_f)                  # pad 8 mm long, contact width ~ filament diameter
    say(f'  M3 thumbscrew at {tq:.2f} N.m finger torque: clamp force F = T/(K d) = {F:.0f} N, contact pressure {p/1e6:.1f} MPa')
say('  Verdict: finger-tight already gives 4-12x the force needed; pressure is below PLA compressive yield (~60 MPa),')
say('  but over-tightening could flatten the filament, so the pad pocket depth should stop the screw (finger tight only).')
say()

say('2. Runout lever drag')
F_sw, mu_roll = 0.5, 0.02                 # microswitch operating force (upper estimate), rolling roller on a pin
say(f'  Lever force on filament <= {F_sw} N; drag ~ mu_roll x F = {mu_roll*F_sw*1000:.0f} mN = {100*mu_roll*F_sw/T:.1f}% of the 5 N set tension. Negligible.')
say()

say('3. Filament path lengths (from the CAD layout, mm)')
x_src, Rm = 190, 150
x_run, x_d, x_whl = x_src + 15, x_src + 135, x_src + 235
x_rcv = 720; x_trv = x_rcv - Rm - 15; x_exit = x_trv - 22; x_clp = x_exit - 29
h, z_p, z_t, fill_rcv = 175, 400, 275, 72
pts = [(x_run, z_p), (x_d - 45, z_p), (x_d - 13, z_p - 45), (x_d + 13, z_p - 45), (x_d + 45, z_p), (x_clp, z_p)]
L_run_clp = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
tail = [(x_clp, z_p), (x_exit, z_p), (x_trv, z_t + 2), (x_rcv - 20, h + fill_rcv)]
L_clp_rcv = sum(math.dist(a, b) for a, b in zip(tail, tail[1:]))
park = L_run_clp - 24                     # tail end parked mid-way along the 47 mm splice span
say(f'  Runout switch to tail clamp: {L_run_clp:.0f} mm; clamp to receiving spool: {L_clp_rcv:.0f} mm; runout to spool: {L_run_clp + L_clp_rcv:.0f} mm')
say(f'  Splice mode creep after the runout trips: about {park:.0f} mm (parks the tail mid-span)')
say(f'  A 2% error in the receiving-radius estimate gives +/-{0.02*park:.0f} mm, well inside the 47 mm span')
say()

say('4. Power budget without the splicer heater (12 V supply)')
R_ph = 1.5                                # ohm per phase, 17HS4401
P_cu = 2 * 1.0**2 * R_ph                  # run current 1.0 A rms
P_mech = 0.13 * (315 * 2 * math.pi / 60)  # worst running torque at max motor speed for 0.5 m/s
P_drv = 1.0
I_wind = (P_cu + P_mech + P_drv) / 12
I_trav = 0.25
I_5v_peak, I_5v_typ, eff = 3.8, 1.0, 0.88
I_buck_peak = I_5v_peak * 5 / eff / 12
I_buck_typ = I_5v_typ * 5 / eff / 12
I_peak = I_wind + I_trav + I_buck_peak
I_typ = I_wind + I_trav + I_buck_typ
say(f'  Winding motor ~{I_wind:.2f} A, traverse ~{I_trav:.2f} A, 5 V rail via buck {I_buck_typ:.2f} A typical / {I_buck_peak:.2f} A with servo stalled')
say(f'  Total from 12 V: {I_typ:.1f} A typical, {I_peak:.1f} A worst case (was about 6 A with the heater)')
say(f'  12 V 5 A supply margin: {5/I_peak:.1f}x worst case')
say('  Breaker: needs a DC rating and a trip current above ~3 A but at or below the 5 A supply: 4 A DC is the right size.')
say('  E-stop NC block: typical 10 A rating, carries < 1.5 A of driver current. OK.')
say()

say('5. Cutting driver power at the e-stop')
Ke = 0.40 / (1.7 * math.sqrt(2))          # V.s/rad, from 0.40 N.m holding torque at 1.7 A
for rpm in (315, 453):
    w = rpm * 2 * math.pi / 60
    say(f'  Back-EMF at {rpm} rpm: {Ke*w:.1f} V peak (below the 12 V rail, so no overvoltage on the TMC2209)')
J = 0.5 * 1.0 * (0.0375**2 + 0.10**2) + 0.002 + 0.0005 + 54e-7 * 4**2   # full receiving spool + spool + gear + reflected rotor
k_f = E_pla * A_f / 1.0                    # filament spring over ~1 m of path
for v in (0.5, 1.13):
    w = v / 0.10
    E = 0.5 * J * w**2
    F_lock = T + math.sqrt(2 * E * k_f)
    say(f'  {v} m/s: receiving side energy {E:.2f} J; if the source were locked, tension peak {F_lock:.0f} N vs {F_break:.0f} N break load')
for r_s in (0.10, 0.035):
    F_slip = 54 * 0.5 * 0.040 / r_s
    say(f'  Brake (54 N pad, mu 0.5, 40 mm drum) slips at {F_slip:.0f} N tension with the source at r = {r_s*1000:.0f} mm, which caps the spike')
say('  Verdict: safe at the 0.5 m/s working speed; at top speed the brake slip is what protects the filament.')
say('  Recommendation: wire a second e-stop contact (or sense the driver supply) to an ESP32 input so the firmware')
say('  also releases the brake and stops the loop when the e-stop is pressed.')

open('v8_checks.txt', 'w').write('\n'.join(out) + '\n')
