# SpoolerLayout.py - basic layout of the smart filament spooler for Fusion 360.
# Builds one component per subsystem so each part can be redesigned on its own.
# All sizes are in mm. Change the numbers in SETTINGS and run the script again
# in a new design to regenerate the layout.

import adsk.core, adsk.fusion, traceback

SETTINGS = {
    'spool_od': 200.0,       # spool outer diameter (Bambu spool, label-based)
    'spool_width': 67.0,     # spool width
    'spool_hole': 55.0,      # spool centre hole diameter
    'shaft_d': 8.0,          # shaft diameter (M8 rod / 608 bearing bore)
    'clearance': 25.0,       # gap between spool rim and base plate
    'base_len': 720.0,       # base plate length (x)
    'base_depth': 240.0,     # base plate depth (y)
    'plate_t': 6.0,          # plate thickness
    'spool_x_margin': 140.0, # spool centre distance from each end of the base
    'gear_small_d': 20.0,    # motor pinion pitch diameter (placeholder)
    'gear_large_d': 60.0,    # spool gear pitch diameter, 3:1 ratio
}

MM = 0.1  # Fusion works in cm internally


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Open a Fusion design (Design workspace) first.')
            return
        root = design.rootComponent
        tbm = adsk.fusion.TemporaryBRepManager.get()
        parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
        s = SETTINGS

        def P(x, y, z):
            return adsk.core.Point3D.create(x * MM, y * MM, z * MM)

        def box(cx, cy, cz, lx, ly, lz):
            obb = adsk.core.OrientedBoundingBox3D.create(
                P(cx, cy, cz),
                adsk.core.Vector3D.create(1, 0, 0),
                adsk.core.Vector3D.create(0, 1, 0),
                lx * MM, ly * MM, lz * MM)
            return tbm.createBox(obb)

        def cyl_y(cx, cz, y0, y1, r0, r1=None):
            # cylinder or cone along the y axis
            if r1 is None:
                r1 = r0
            return tbm.createCylinderOrCone(P(cx, y0, cz), r0 * MM, P(cx, y1, cz), r1 * MM)

        def cyl_z(cx, cy, z0, z1, r):
            return tbm.createCylinderOrCone(P(cx, cy, z0), r * MM, P(cx, cy, z1), r * MM)

        def cut(target, tool):
            tbm.booleanOperation(target, tool, adsk.fusion.BooleanTypes.DifferenceBooleanType)
            return target

        state = {'use_components': True}

        def put(comp, name, bodies):
            if parametric:
                bf = comp.features.baseFeatures.add()
                bf.startEdit()
                made = [comp.bRepBodies.add(b, bf) for b in bodies]
                bf.finishEdit()
            else:
                made = [comp.bRepBodies.add(b) for b in bodies]
            for i, b in enumerate(made):
                try:
                    b.name = name if len(made) == 1 else '%s %d' % (name, i + 1)
                except Exception:
                    pass
            return made

        def add(name, bodies, opacity=None):
            # One component per subsystem; Part designs that block components get named bodies instead.
            if state['use_components']:
                try:
                    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                    comp = occ.component
                    comp.name = name
                    put(comp, name, bodies)
                    if opacity is not None:
                        try:
                            occ.opacity = opacity
                        except Exception:
                            pass
                    return
                except Exception:
                    state['use_components'] = False
            made = put(root, name, bodies)
            if opacity is not None:
                for b in made:
                    try:
                        b.opacity = opacity
                    except Exception:
                        pass

        # ---- derived layout ----
        R = s['spool_od'] / 2.0
        r_hole = s['spool_hole'] / 2.0
        r_shaft = s['shaft_d'] / 2.0
        t = s['plate_t']
        h = R + s['clearance']                 # shaft height above base
        z_path = h + R                          # filament path height (top of spool)
        x_src = s['spool_x_margin']             # source spool centre
        x_rcv = s['base_len'] - s['spool_x_margin']  # receiving spool centre
        y_up = -60.0                            # front face of the upright plates
        y_sp0 = y_up + 10.0                     # spool back face
        y_sp1 = y_sp0 + s['spool_width']        # spool front face
        y_c = (y_sp0 + y_sp1) / 2.0             # filament path line (spool mid-width)
        y_base0 = -140.0
        y_base1 = y_base0 + s['base_depth']

        # ---- 1. base plate ----
        add('Base plate', [box(s['base_len'] / 2.0, (y_base0 + y_base1) / 2.0, -t / 2.0,
                                s['base_len'], s['base_depth'], t)])

        # ---- 2. spool uprights, shafts and cone hubs ----
        def spool_station(name, xc, shaft_back):
            up_h = h + 30.0
            upright = box(xc, y_up - t / 2.0, up_h / 2.0, 70.0, t, up_h)
            cut(upright, cyl_y(xc, h, y_up - t - 1, y_up + 1, r_shaft + 0.5))
            shaft = cyl_y(xc, h, shaft_back, y_sp1 + 25.0, r_shaft)
            cone_back = cyl_y(xc, h, y_sp0 - 8.0, y_sp0 + 7.0, r_hole + 12.0, r_hole - 8.0)
            cone_front = cyl_y(xc, h, y_sp1 + 8.0, y_sp1 - 7.0, r_hole + 12.0, r_hole - 8.0)
            for c in (cone_back, cone_front):
                cut(c, cyl_y(xc, h, y_sp0 - 20.0, y_sp1 + 20.0, r_shaft + 0.2))
            add(name + ' - upright', [upright])
            add(name + ' - shaft', [shaft])
            add(name + ' - cone hubs', [cone_back, cone_front])

        spool_station('Source spool', x_src, y_up - t - 5.0)
        spool_station('Receiving spool', x_rcv, y_up - 40.0)

        # ---- 3. reference spools (envelope only, see-through) ----
        for name, xc in (('REF source spool', x_src), ('REF receiving spool', x_rcv)):
            spool = cyl_y(xc, h, y_sp0, y_sp1, R)
            cut(spool, cyl_y(xc, h, y_sp0 - 1, y_sp1 + 1, r_hole))
            add(name, [spool], opacity=0.25)

        # ---- 4. drive: NEMA 17 motor and 3:1 gears behind the receiving upright ----
        gear_y0, gear_y1 = y_up - 30.0, y_up - 22.0
        centre_dist = (s['gear_large_d'] + s['gear_small_d']) / 2.0
        x_motor = x_rcv + centre_dist
        big_gear = cyl_y(x_rcv, h, gear_y0, gear_y1, s['gear_large_d'] / 2.0)
        pinion = cyl_y(x_motor, h, gear_y0, gear_y1, s['gear_small_d'] / 2.0)
        motor = box(x_motor, gear_y0 - 6.0 - 20.0, h, 42.0, 40.0, 42.0)
        motor_shaft = cyl_y(x_motor, h, gear_y0 - 6.0, gear_y1, 2.5)
        add('Drive - 3:1 gears (placeholder)', [big_gear, pinion])
        add('Drive - NEMA 17 motor', [motor, motor_shaft])

        # ---- 5. brake servo on the source spool (placeholder) ----
        add('Tension brake servo (placeholder)',
            [box(x_src + 60.0, y_up - t - 15.0, h - 40.0, 23.0, 12.0, 29.0)])

        # ---- 6. filament path stages, each on a post ----
        gap0 = x_src + R
        gap1 = x_rcv - R
        span = gap1 - gap0
        x_run = gap0 + 0.12 * span
        x_dan = gap0 + 0.32 * span
        x_whl = gap0 + 0.52 * span
        x_spl = gap0 + 0.72 * span
        x_trv = gap0 + 0.92 * span

        def post(x, top):
            return box(x, y_c - 15.0, top / 2.0, 8.0, 8.0, top)

        add('Runout switch', [box(x_run, y_c, z_path, 20.0, 16.0, 14.0), post(x_run, z_path - 7.0)])

        dancer_arm = box(x_dan + 20.0, y_c - 8.0, z_path - 18.0, 50.0, 6.0, 6.0)
        dancer_pulley = cyl_y(x_dan + 40.0, z_path - 12.0, y_c - 3.0, y_c + 3.0, 12.5)
        add('Dancer arm (tension sensing)', [dancer_arm, dancer_pulley, post(x_dan, z_path - 21.0)])

        wheel = cyl_y(x_whl, z_path - 10.0, y_c - 3.0, y_c + 3.0, 10.0)
        idler = cyl_y(x_whl, z_path + 6.0, y_c - 3.0, y_c + 3.0, 6.0)
        encoder = box(x_whl, y_c - 10.0, z_path - 10.0, 20.0, 6.0, 20.0)
        add('Measuring wheel + AS5600', [wheel, idler, encoder, post(x_whl, z_path - 20.0)])

        add('Splicer (stretch goal)', [box(x_spl, y_c, z_path, 40.0, 20.0, 20.0), post(x_spl, z_path - 10.0)])

        rail = cyl_y(x_trv, z_path, y_sp0 - 15.0, y_sp1 + 15.0, 4.0)
        carriage = box(x_trv, y_c, z_path, 14.0, 14.0, 14.0)
        trv_motor = cyl_y(x_trv, z_path, y_sp0 - 35.0, y_sp0 - 16.0, 14.0)
        add('Traverse (rail, carriage, motor)',
            [rail, carriage, trv_motor, box(x_trv, y_sp0 - 15.0, (z_path - 4.0) / 2.0, 8.0, 8.0, z_path - 4.0)])

        # ---- 7. electronics box and touchscreen ----
        ebox_x = x_rcv - 20.0
        ebox_y = y_base1 - 35.0
        add('Electronics box (ESP32, drivers)', [box(ebox_x, ebox_y, 17.5, 130.0, 60.0, 35.0)])
        add('Touchscreen 2.8in', [box(ebox_x, y_base1 - 8.0, 35.0 + 30.0, 75.0, 5.0, 55.0)])

        app.activeViewport.fit()
        ui.messageBox('Spooler layout built (%d components, %d loose bodies).\n'
                      'Sizes come from SETTINGS at the top of the script.'
                      % (root.occurrences.count, root.bRepBodies.count))
    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
