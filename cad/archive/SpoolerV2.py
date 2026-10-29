# SpoolerV2.py - upgrades the SpoolerLayout model (run it on the saved v1 design).
# 1. Replaces the two placeholder spool blocks with realistic 1 kg spools
#    (flanges with windows, core, bore, ribs, wound filament with ridges).
# 2. Adds the filament strand along the path.
# 3. Renames every body and gives it a physical material and a colour.
# Colours and materials are in LOOKS at the top, so they are easy to change.

import adsk.core, adsk.fusion, traceback, math

# ---- spool and layout numbers (must match SpoolerLayout SETTINGS) ----
S = {
    'spool_od': 200.0, 'spool_width': 67.0, 'spool_hole': 55.0, 'core_d': 122.0,
    'flange_t': 3.0, 'clearance': 25.0, 'base_len': 720.0, 'spool_x_margin': 140.0,
    'fill_source': 95.0,     # filament outer radius on the source spool (full, label-based)
    'fill_receive': 72.0,    # filament outer radius on the receiving spool (part wound)
    'filament_d': 1.75,
}

# ---- looks: key -> (material candidates, appearance candidates, RGB or None) ----
LOOKS = {
    'frame':    (['PLA', 'Polylactic', 'Plastic'], ['Plastic - Matte', 'Plastic - Glossy', 'Plastic'], (55, 58, 62)),
    'moving':   (['PLA', 'Polylactic', 'Plastic'], ['Plastic - Matte', 'Plastic - Glossy', 'Plastic'], (235, 110, 25)),
    'housing':  (['PLA', 'Polylactic', 'Plastic'], ['Plastic - Matte', 'Plastic - Glossy', 'Plastic'], (205, 207, 210)),
    'spool':    (['ABS Plastic', 'ABS', 'Plastic'], ['Plastic - Glossy', 'Plastic - Matte', 'Plastic'], (22, 22, 24)),
    'filament': (['PLA', 'Polylactic', 'Plastic'], ['Plastic - Glossy', 'Plastic - Matte', 'Plastic'], (0, 145, 190)),
    'steel':    (['Steel', 'Steel, Mild', 'Stainless Steel'], ['Steel - Satin', 'Steel - Polished', 'Steel'], None),
    'alu':      (['Aluminum 6061', 'Aluminum', 'Aluminium'], ['Aluminum - Satin', 'Aluminum - Polished', 'Aluminum'], None),
    'motor':    (['Steel', 'Steel, Mild'], ['Plastic - Matte', 'Plastic'], (30, 30, 32)),
    'black':    (['ABS Plastic', 'ABS', 'Plastic'], ['Plastic - Matte', 'Plastic'], (25, 25, 27)),
    'pcb':      (['FR-4', 'Epoxy', 'Plastic'], ['Plastic - Glossy', 'Plastic'], (20, 110, 60)),
    'screen':   (['Glass', 'Plastic'], ['Plastic - Glossy', 'Plastic'], (10, 12, 18)),
}

# ---- (v1 body number, old name, new name, look). Fusion keeps default names
#      Body1..Body33 in build order, so the number is the reliable key. ----
RENAME = [
    (1, 'Base plate', 'Base plate', 'frame'),
    (2, 'Source spool - upright', 'Upright - source', 'frame'),
    (6, 'Receiving spool - upright', 'Upright - receiving', 'frame'),
    (3, 'Source spool - shaft', 'Shaft M8 - source', 'steel'),
    (7, 'Receiving spool - shaft', 'Shaft M8 - receiving', 'steel'),
    (4, 'Source spool - cone hubs 1', 'Cone hub - source back', 'moving'),
    (5, 'Source spool - cone hubs 2', 'Cone hub - source front', 'moving'),
    (8, 'Receiving spool - cone hubs 1', 'Cone hub - receiving back', 'moving'),
    (9, 'Receiving spool - cone hubs 2', 'Cone hub - receiving front', 'moving'),
    (12, 'Drive - 3:1 gears (placeholder) 1', 'Gear 60 mm - spool', 'moving'),
    (13, 'Drive - 3:1 gears (placeholder) 2', 'Gear 20 mm - motor pinion', 'moving'),
    (14, 'Drive - NEMA 17 motor 1', 'NEMA 17 - winding motor', 'motor'),
    (15, 'Drive - NEMA 17 motor 2', 'NEMA 17 - shaft', 'steel'),
    (16, 'Tension brake servo (placeholder)', 'Servo - tension brake', 'black'),
    (17, 'Runout switch 1', 'Runout switch', 'black'),
    (18, 'Runout switch 2', 'Post - runout switch', 'frame'),
    (19, 'Dancer arm (tension sensing) 1', 'Dancer arm', 'moving'),
    (20, 'Dancer arm (tension sensing) 2', 'Dancer pulley', 'moving'),
    (21, 'Dancer arm (tension sensing) 3', 'Post - dancer', 'frame'),
    (22, 'Measuring wheel + AS5600 1', 'Measuring wheel', 'alu'),
    (23, 'Measuring wheel + AS5600 2', 'Measuring idler', 'moving'),
    (24, 'Measuring wheel + AS5600 3', 'AS5600 encoder board', 'pcb'),
    (25, 'Measuring wheel + AS5600 4', 'Post - measuring wheel', 'frame'),
    (26, 'Splicer (stretch goal) 1', 'Splicer heater block', 'alu'),
    (27, 'Splicer (stretch goal) 2', 'Post - splicer', 'frame'),
    (28, 'Traverse (rail, carriage, motor) 1', 'Traverse rod M8', 'steel'),
    (29, 'Traverse (rail, carriage, motor) 2', 'Traverse carriage', 'moving'),
    (30, 'Traverse (rail, carriage, motor) 3', '28BYJ-48 traverse motor', 'steel'),
    (31, 'Traverse (rail, carriage, motor) 4', 'Post - traverse', 'frame'),
    (32, 'Electronics box (ESP32, drivers)', 'Electronics enclosure', 'housing'),
    (33, 'Touchscreen 2.8in', 'Touchscreen 2.8in', 'screen'),
]

MM = 0.1


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Open the spooler design first.')
            return
        root = design.rootComponent
        tbm = adsk.fusion.TemporaryBRepManager.get()
        parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
        UNION = adsk.fusion.BooleanTypes.UnionBooleanType
        DIFF = adsk.fusion.BooleanTypes.DifferenceBooleanType

        def P(x, y, z):
            return adsk.core.Point3D.create(x * MM, y * MM, z * MM)

        def box(cx, cy, cz, lx, ly, lz):
            obb = adsk.core.OrientedBoundingBox3D.create(
                P(cx, cy, cz), adsk.core.Vector3D.create(1, 0, 0),
                adsk.core.Vector3D.create(0, 1, 0), lx * MM, ly * MM, lz * MM)
            return tbm.createBox(obb)

        def cyl_y(cx, cz, y0, y1, r):
            return tbm.createCylinderOrCone(P(cx, y0, cz), r * MM, P(cx, y1, cz), r * MM)

        def tube_y(cx, cz, y0, y1, r_out, r_in):
            t = cyl_y(cx, cz, y0, y1, r_out)
            tbm.booleanOperation(t, cyl_y(cx, cz, y0 - 1, y1 + 1, r_in), DIFF)
            return t

        def rod(p0, p1, r):
            return tbm.createCylinderOrCone(P(*p0), r * MM, P(*p1), r * MM)

        def add_bodies(named):
            # named: list of (name, temp body). Returns dict name -> real body.
            out = {}
            if parametric:
                bf = root.features.baseFeatures.add()
                bf.startEdit()
                for _, b in named:
                    root.bRepBodies.add(b, bf)
                bf.finishEdit()
                made = [bf.bodies.item(i) for i in range(bf.bodies.count)]
            else:
                made = [root.bRepBodies.add(b) for _, b in named]
            for (n, _), b in zip(named, made):
                try:
                    b.name = n
                except Exception:
                    pass
                out[n] = b
            return out

        # ---- library lookups (cached) ----
        mat_index, app_index = {}, {}
        for i in range(app.materialLibraries.count):
            lib = app.materialLibraries.item(i)
            try:
                for j in range(lib.materials.count):
                    m = lib.materials.item(j)
                    mat_index.setdefault(m.name.lower(), m)
            except Exception:
                pass
            try:
                for j in range(lib.appearances.count):
                    a = lib.appearances.item(j)
                    app_index.setdefault(a.name.lower(), a)
            except Exception:
                pass

        def find(index, cands):
            for c in cands:
                if c.lower() in index:
                    return index[c.lower()]
            for c in cands:
                for k in sorted(index):
                    if k.startswith(c.lower()):
                        return index[k]
            for c in cands:
                for k in sorted(index):
                    if c.lower() in k:
                        return index[k]
            return None

        def set_colour(appearance, rgb):
            col = adsk.core.Color.create(rgb[0], rgb[1], rgb[2], 255)
            props = appearance.appearanceProperties
            for pid in ('Color', 'opaque_albedo', 'generic_diffuse', 'metal_f0', 'layered_diffuse'):
                try:
                    p = adsk.core.ColorProperty.cast(props.itemByName(pid)) or adsk.core.ColorProperty.cast(props.itemById(pid))
                except Exception:
                    p = None
                if p:
                    p.value = col
                    return True
            for i in range(props.count):
                p = adsk.core.ColorProperty.cast(props.item(i))
                if p and not p.isReadOnly:
                    p.value = col
                    return True
            return False

        looks, used = {}, []
        for key, (mcands, acands, rgb) in LOOKS.items():
            mat = find(mat_index, mcands)
            base = find(app_index, acands)
            appear = None
            if base:
                nm = 'Spooler - ' + key
                appear = design.appearances.itemByName(nm)
                if not appear:
                    appear = design.appearances.addByCopy(base, nm)
                    if rgb:
                        set_colour(appear, rgb)
            looks[key] = (mat, appear)
            used.append('%s: %s / %s' % (key, mat.name if mat else '-', base.name if base else '-'))

        def apply(body, key):
            mat, appear = looks[key]
            if mat:
                try:
                    body.material = mat
                except Exception:
                    pass
            if appear:
                try:
                    body.appearance = appear
                except Exception:
                    pass

        # ---- 1. remove placeholder spools ----
        for n, num in (('REF source spool', 10), ('REF receiving spool', 11)):
            b = root.bRepBodies.itemByName(n) or root.bRepBodies.itemByName('Body%d' % num)
            if b:
                b.deleteMe()

        # ---- 2. realistic spools ----
        R = S['spool_od'] / 2.0
        h = R + S['clearance']
        z_path = h + R
        x_src = S['spool_x_margin']
        x_rcv = S['base_len'] - S['spool_x_margin']
        y_sp0 = -60.0 + 10.0
        y_sp1 = y_sp0 + S['spool_width']
        y_c = (y_sp0 + y_sp1) / 2.0
        ft = S['flange_t']
        r_hole = S['spool_hole'] / 2.0
        r_core = S['core_d'] / 2.0
        r_bore = r_hole + 3.5

        def spool(xc):
            # flanges with six windows and three filament-clip holes
            fl = []
            for y0, y1 in ((y_sp0, y_sp0 + ft), (y_sp1 - ft, y_sp1)):
                f = tube_y(xc, h, y0, y1, R, r_hole)
                for k in range(6):
                    a = math.radians(30 + 60 * k)
                    tbm.booleanOperation(f, cyl_y(xc + 78 * math.cos(a), h + 78 * math.sin(a), y0 - 1, y1 + 1, 11.0), DIFF)
                for k in range(3):
                    a = math.radians(90 + 120 * k)
                    tbm.booleanOperation(f, cyl_y(xc + 92 * math.cos(a), h + 92 * math.sin(a), y0 - 1, y1 + 1, 1.2), DIFF)
                fl.append(f)
            body = fl[0]
            tbm.booleanOperation(body, fl[1], UNION)
            yi0, yi1 = y_sp0 + ft - 0.5, y_sp1 - ft + 0.5
            tbm.booleanOperation(body, tube_y(xc, h, yi0, yi1, r_core, r_core - 3.0), UNION)
            tbm.booleanOperation(body, tube_y(xc, h, yi0, yi1, r_bore, r_hole), UNION)
            mid = (r_bore + r_core - 3.0) / 2.0
            ln = (r_core - 3.0) - r_bore + 2.0
            for dx, dz, lx, lz in ((mid, 0, ln, 3), (-mid, 0, ln, 3), (0, mid, 3, ln), (0, -mid, 3, ln)):
                tbm.booleanOperation(body, box(xc + dx, y_c, h + dz, lx, yi1 - yi0, lz), UNION)
            return body

        def wound(xc, r_fill):
            y0, y1 = y_sp0 + ft, y_sp1 - ft
            w = tube_y(xc, h, y0, y1, r_fill - 0.6, r_core)
            pitch = S['filament_d']
            n = int((y1 - y0) / pitch)
            for i in range(n):
                a = y0 + i * pitch + 0.2
                ring = tube_y(xc, h, a, a + 0.9, r_fill, r_fill - 1.2)
                tbm.booleanOperation(w, ring, UNION)
            return w

        r_strand = S['filament_d'] / 2.0
        x_trv = (x_src + R) + 0.92 * ((x_rcv - R) - (x_src + R))
        top_src = h + S['fill_source']
        top_rcv = h + S['fill_receive']
        strand = rod((x_src, y_c, top_src), (x_trv, y_c, z_path), r_strand)
        tbm.booleanOperation(strand, rod((x_trv, y_c, z_path), (x_rcv, y_c, top_rcv), r_strand), UNION)

        new_names = ['Spool - source (1 kg)', 'Filament - source (full)', 'Spool - receiving',
                     'Filament - receiving (part wound)', 'Filament strand']
        earlier = [root.bRepBodies.itemByName(n) or root.bRepBodies.itemByName('Body%d' % (34 + i))
                   for i, n in enumerate(new_names)]
        if all(earlier):
            new = {}
            for n, b in zip(new_names, earlier):
                b.name = n
                new[n] = b
        else:
          new = add_bodies([
              ('Spool - source (1 kg)', spool(x_src)),
              ('Filament - source (full)', wound(x_src, S['fill_source'])),
              ('Spool - receiving', spool(x_rcv)),
              ('Filament - receiving (part wound)', wound(x_rcv, S['fill_receive'])),
              ('Filament strand', strand),
          ])
        apply(new['Spool - source (1 kg)'], 'spool')
        apply(new['Spool - receiving'], 'spool')
        for n in ('Filament - source (full)', 'Filament - receiving (part wound)', 'Filament strand'):
            apply(new[n], 'filament')

        # ---- 3. rename and colour the existing bodies ----
        missing = []
        for num, old, newname, key in RENAME:
            b = (root.bRepBodies.itemByName(old) or root.bRepBodies.itemByName('Body%d' % num)
                 or root.bRepBodies.itemByName(newname))
            if not b:
                missing.append(old)
                continue
            try:
                b.name = newname
            except Exception:
                pass
            apply(b, key)

        app.activeViewport.fit()
        msg = 'Spooler v2 done: %d bodies.\n\nMaterial / appearance used:\n%s' % (
            root.bRepBodies.count, '\n'.join(used))
        if missing:
            msg += '\n\nNot found: ' + ', '.join(missing)
        ui.messageBox(msg)
    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
