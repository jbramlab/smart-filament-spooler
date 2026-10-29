# SpoolerTensionModel.py - runs the spooler tension model inside Fusion's Python (no MATLAB needed).
# Same equations and parameters as spooler_params.m / spooler_sim_ode.m.
# Writes results CSV + an HTML page with plots to Videos\Spooler Animation\Tension model.

import adsk.core, traceback, math, os

P = dict(d=1.75e-3, rho=1240.0, w=0.061, k_pack=0.80, rc=0.061, J_shell=0.0025, b_s=1e-4,
         mu=0.5, r_d=0.040, Fmax=54.0, tau_srv=0.08, eps_w=0.005,
         k_sp=670.0, c_sp=5.0, xf=0.030, xmax=0.030, x_set=0.015,
         N=68.0 / 17.0, eta=0.9, v_run=0.5, t_ramp=3.0, t_stop=22.0, T_end=30.0,
         Kp=1500.0, Ki=6000.0, Kd=250.0)
P['lam'] = P['rho'] * math.pi * P['d'] ** 2 / 4
P['L_full'] = 1.0 / P['lam']
P['T_set'] = P['k_sp'] * (P['xf'] - P['x_set']) / 2

OUT = os.path.join(os.path.expanduser('~'), 'Videos', 'Spooler Animation', 'Tension model')


def radius(L):
    return math.sqrt(P['rc'] ** 2 + max(L, 0.0) * P['d'] ** 2 / (4 * P['w'] * P['k_pack']))


def simulate(L_start, dt=2e-4):
    ws, x, Ls, Lr, I = 0.0, P['x_set'], 0.0, 0.0, 0.0
    Fb = P['T_set'] * radius(L_start) / (P['mu'] * P['r_d'])
    n = int(round(P['T_end'] / dt))
    log, tau_peak = [], 0.0
    for i in range(n):
        t = i * dt
        if t < P['t_ramp']:
            v = P['v_run'] * t / P['t_ramp']
        elif t < P['t_stop']:
            v = P['v_run']
        elif t < P['t_stop'] + P['t_ramp']:
            v = P['v_run'] * (1 - (t - P['t_stop']) / P['t_ramp'])
        else:
            v = 0.0
        Lsrc = max(L_start - Ls, 0.0)
        rs, rr = radius(Lsrc), radius(Lr)
        Js = P['J_shell'] + 0.5 * P['lam'] * Lsrc * (P['rc'] ** 2 + rs ** 2)
        vs = ws * rs
        xd = (vs - v) / 2
        T = max(0.0, (P['k_sp'] * (P['xf'] - x) - P['c_sp'] * xd) / 2)
        e = x - P['x_set']
        I += e * dt
        Fcmd = P['T_set'] * rs / (P['mu'] * P['r_d']) + P['Kp'] * e + P['Ki'] * I + P['Kd'] * xd
        if Fcmd > P['Fmax'] or Fcmd < 0:
            Fcmd = min(max(Fcmd, 0.0), P['Fmax'])
            I -= e * dt
        Fb += (Fcmd - Fb) / P['tau_srv'] * dt
        tb = P['mu'] * Fb * P['r_d'] * math.tanh(ws / P['eps_w'])
        wsd = (T * rs - tb - P['b_s'] * ws) / Js
        tau_peak = max(tau_peak, T * rr / (P['N'] * P['eta']))
        if i % 50 == 0:                       # keep every 10 ms
            log.append((t, v, vs, T, x * 1000, Fb, rs * 1000))
        ws += wsd * dt
        x = min(max(x + xd * dt, 0.0), P['xmax'])
        Ls += vs * dt
        Lr += v * dt
    return log, tau_peak


def svg_panel(series, ylabel, y0, y1, w=820, h=190, ref=None):
    L, R, Tp, B = 60, 20, 12, 30
    X = lambda t: L + (w - L - R) * t / P['T_end']
    Y = lambda y: Tp + (h - Tp - B) * (1 - (y - y0) / (y1 - y0))
    s = ['<svg viewBox="0 0 %d %d" width="%d" height="%d" style="background:#fff">' % (w, h, w, h)]
    for k in range(6):
        yy = y0 + (y1 - y0) * k / 5
        s.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#e5e5e5"/>' % (L, w - R, Y(yy), Y(yy)))
        s.append('<text x="%d" y="%.1f" font-size="11" text-anchor="end" fill="#555">%.2f</text>' % (L - 6, Y(yy) + 4, yy))
    for k in range(0, int(P['T_end']) + 1, 5):
        s.append('<text x="%.1f" y="%d" font-size="11" text-anchor="middle" fill="#555">%d s</text>' % (X(k), h - 10, k))
    if ref is not None:
        s.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#000" stroke-dasharray="4 4"/>' % (L, w - R, Y(ref), Y(ref)))
    for pts, col, dash in series:
        d = ' '.join('%.1f,%.1f' % (X(t), Y(min(max(y, y0), y1))) for t, y in pts)
        s.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="1.5" %s/>' % (d, col, 'stroke-dasharray="5 4"' if dash else ''))
    s.append('<text x="14" y="%d" font-size="12" fill="#222" transform="rotate(-90 14 %d)" text-anchor="middle">%s</text>' % (h // 2, h // 2, ylabel))
    s.append('</svg>')
    return '\n'.join(s)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        os.makedirs(OUT, exist_ok=True)
        cases = [(P['L_full'], 'Full source spool', '#1f77b4'), (0.03 * P['L_full'], 'Nearly empty source', '#ff7f0e')]
        results, lines = [], []
        for L0, name, col in cases:
            log, tau_peak = simulate(L0)
            run_ = [r for r in log if 5 < r[0] < 21]
            summ = dict(name=name, col=col, r0=log[0][6], tmin=min(r[3] for r in run_), tmax=max(r[3] for r in run_),
                        tall_min=min(r[3] for r in log), tall_max=max(r[3] for r in log),
                        xmin=min(r[4] for r in log), xmax=max(r[4] for r in log),
                        fmin=min(r[5] for r in log), fmax=max(r[5] for r in log), tau=tau_peak)
            results.append((log, summ))
            lines.append('%s (start radius %.1f mm): tension while running %.2f-%.2f N (whole run %.2f-%.2f N), '
                         'dancer %.1f-%.1f mm, brake %.1f-%.1f N, peak motor torque %.3f N m' % (
                             name, summ['r0'], summ['tmin'], summ['tmax'], summ['tall_min'], summ['tall_max'],
                             summ['xmin'], summ['xmax'], summ['fmin'], summ['fmax'], summ['tau']))
            with open(os.path.join(OUT, name.replace(' ', '_') + '.csv'), 'w') as fh:
                fh.write('time_s,target_speed_m_s,source_speed_m_s,tension_N,dancer_mm,brake_force_N,source_radius_mm\n')
                for r in log:
                    fh.write('%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.2f\n' % r)

        panels = [
            ('Line speed (m/s), dashed = source pay-out', 0, 0.6, [(1, False), (2, True)], None),
            ('Tension (N), dotted = 5 N target', 3.8, 5.4, [(3, False)], P['T_set']),
            ('Dancer position (mm)', 13, 19, [(4, False)], None),
            ('Brake force (N)', 0, 56, [(5, False)], None)]
        html = ['<html><head><meta charset="utf-8"><title>Spooler tension model</title></head>'
                '<body style="font-family:Segoe UI,Arial;margin:24px;color:#222">',
                '<h2>Spooler tension model - run in Fusion on this PC</h2>',
                '<p>Start, run at %.1f m/s, stop. Target tension %.2f N. Blue = full source spool, orange = nearly empty.</p>'
                % (P['v_run'], P['T_set'])]
        html += ['<p>%s</p>' % l for l in lines]
        for title, y0, y1, cols, ref in panels:
            series = []
            for log, summ in results:
                for c, dash in cols:
                    series.append(([(r[0], r[c]) for r in log], summ['col'], dash))
            html.append('<h4 style="margin:18px 0 4px">%s</h4>' % title)
            html.append(svg_panel(series, '', y0, y1, ref=ref))
        html.append('</body></html>')
        page = os.path.join(OUT, 'Spooler tension model results.html')
        with open(page, 'w', encoding='utf-8') as fh:
            fh.write('\n'.join(html))
        ui.messageBox('Spooler tension model finished.\n\n' + '\n\n'.join(lines) + '\n\nResults: ' + OUT)
    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
