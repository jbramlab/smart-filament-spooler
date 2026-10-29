# SpoolerAssembly.py - Smart Filament Spooler, detailed assembly for Fusion.
#
# Creates a NEW design with one component per part, realistic geometry, materials,
# colours and working joints (spools, gears, brake lever, dancer, measuring wheel
# turn; traverse carriage slides). Sized for the full design envelope
# (spools 140-300 mm OD, 40-110 mm wide, 25-80 mm centre hole).
#
# Units: mm. Axes: X along the machine, Y towards the front, Z up.
# To change the design, edit P below and run the script again (it makes a new design).
# Geometry is a starting framework: check fits and tolerances before printing.
# v2: involute gears (17T/68T, module 1.5, 20 deg), machine front = Fusion FRONT view,
# printable splits for parts over the bed size, bolt holes, ghost 300 mm spools,
# interference check and a print list (PrintList.csv next to this script).

import adsk.core, adsk.fusion, traceback, math, os

REBUILD_ACTIVE = True   # rebuild inside the open spooler design (saves as a new version)
FLIP_Y = True           # machine front (screen side) faces Fusion's FRONT view
BED = 250.0             # printer bed limit (mm) - Bambu printers are 256 mm

P = {
    # design envelope
    'od_max': 300.0, 'w_max': 110.0, 'clear': 25.0,
    # reference spool shown in the model (Bambu 1 kg)
    'od': 200.0, 'w': 67.0, 'hole': 55.0, 'core': 122.0, 'flange_t': 3.0,
    'fill_src': 95.0, 'fill_rcv': 72.0, 'fil_d': 1.75,
    # mechanics
    'gap': 230.0,          # space between the two largest spools
    'shaft_d': 8.0, 'module': 1.5, 'z_pinion': 17, 'z_gear': 68, 'gear_w': 8.0,
    'base_t': 9.0, 'base_d': 340.0, 'up_t': 12.0, 'up_w': 100.0,
    'drum_r': 40.0, 'lever_ratio_len': 130.0,
    'backlash': 0.25,      # extra gear centre distance for printed gears (mm)
}

MM = 0.1


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        notes = []
        design = adsk.fusion.Design.cast(app.activeProduct)
        rebuilt = False
        if REBUILD_ACTIVE and design:
            r0 = design.rootComponent
            names = [r0.occurrences.item(i).component.name for i in range(r0.occurrences.count)]
            if 'Frame' in names:
                for occ in [r0.occurrences.item(i) for i in range(r0.occurrences.count)]:
                    occ.deleteMe()
                rebuilt = True
                notes.append('Rebuilt inside the open design')
        if not rebuilt:
            app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
            design = adsk.fusion.Design.cast(app.activeProduct)
        try:
            design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        except Exception:
            pass
        try:
            design.designIntent = adsk.fusion.DesignIntentTypes.AssemblyDesignIntentType
            notes.append('Design intent set to Assembly')
        except Exception as e:
            notes.append('Could not set design intent (%s)' % e.__class__.__name__)
        try:
            design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        except Exception:
            pass
        root = design.rootComponent
        tbm = adsk.fusion.TemporaryBRepManager.get()
        parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
        UNION = adsk.fusion.BooleanTypes.UnionBooleanType
        DIFF = adsk.fusion.BooleanTypes.DifferenceBooleanType
        SY = -1.0 if FLIP_Y else 1.0

        def V(dx, dy, dz):
            return adsk.core.Vector3D.create(dx, SY * dy, dz)

        # ---------------- geometry helpers (all mm, world coords) ----------------
        def P3(x, y, z):
            return adsk.core.Point3D.create(x * MM, SY * y * MM, z * MM)

        def box(x0, x1, y0, y1, z0, z1):
            obb = adsk.core.OrientedBoundingBox3D.create(
                P3((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), V(1, 0, 0), V(0, 1, 0),
                abs(x1 - x0) * MM, abs(y1 - y0) * MM, abs(z1 - z0) * MM)
            return tbm.createBox(obb)

        def obox(c, ldir, wdir, l, w, h):
            obb = adsk.core.OrientedBoundingBox3D.create(P3(*c), V(*ldir), V(*wdir), l * MM, w * MM, h * MM)
            return tbm.createBox(obb)

        def cyl(p0, p1, r0, r1=None):
            return tbm.createCylinderOrCone(P3(*p0), r0 * MM, P3(*p1), (r0 if r1 is None else r1) * MM)

        def cy(x, z, y0, y1, r, r1=None):        # along Y
            return cyl((x, y0, z), (x, y1, z), r, r1)

        def cx(y, z, x0, x1, r):                 # along X
            return cyl((x0, y, z), (x1, y, z), r)

        def cz(x, y, z0, z1, r):                 # along Z
            return cyl((x, y, z0), (x, y, z1), r)

        def tube_y(x, z, y0, y1, ro, ri):
            t = cy(x, z, y0, y1, ro)
            tbm.booleanOperation(t, cy(x, z, y0 - 1, y1 + 1, ri), DIFF)
            return t

        def sphere(c, r):
            return tbm.createSphere(P3(*c), r * MM)

        def U(a, *bs):
            for b in bs:
                tbm.booleanOperation(a, b, UNION)
            return a

        def D(a, *bs):
            for b in bs:
                tbm.booleanOperation(a, b, DIFF)
            return a

        def rod_path(pts, r):
            body = None
            for a, b in zip(pts[:-1], pts[1:]):
                seg = cyl(a, b, r)
                body = seg if body is None else U(body, seg)
            for p in pts[1:-1]:
                U(body, sphere(p, r))
            return body

        def gear_y(x, z, y0, y1, teeth, m, bore):
            rp = m * teeth / 2.0
            rr, rt = rp - 1.25 * m, rp + m
            g = cy(x, z, y0, y1, rr)
            for k in range(teeth):
                a = 2 * math.pi * k / teeth
                c, s = math.cos(a), math.sin(a)
                mid = (rr + rt) / 2 - 0.3
                g = U(g, obox((x + mid * c, (y0 + y1) / 2, z + mid * s), (c, 0, s), (-s, 0, c),
                              rt - rr + 0.6, 1.45 * m, y1 - y0))
            return D(g, cy(x, z, y0 - 1, y1 + 1, bore))

        def involute_outline(teeth, m, pa=20.0, phase=0.0):
            # closed outline (list of (u, v) in mm about the gear centre), counter-clockwise
            a = math.radians(pa)
            rp = m * teeth / 2.0
            rb = rp * math.cos(a)
            ra = rp + m
            rf = rp - 1.25 * m
            inv = lambda al: math.tan(al) - al
            half = math.pi / (2 * teeth) + inv(a)
            def delta(r):
                if r <= rb:
                    return half
                return half - inv(math.acos(rb / r))
            r0 = max(rf, rb)
            radii = [r0 + (ra - r0) * k / 6.0 for k in range(7)]
            pts = []
            for t in range(teeth):
                c = phase + 2 * math.pi * t / teeth
                if rf < rb:
                    pts.append((rf, c - delta(rb)))
                for r in radii:
                    pts.append((r, c - delta(r)))
                pts.append((ra, c))
                for r in reversed(radii):
                    pts.append((r, c + delta(r)))
                if rf < rb:
                    pts.append((rf, c + delta(rb)))
                nxt = c + 2 * math.pi / teeth
                mid = (c + delta(rf if rf >= rb else rb) + nxt - delta(rf if rf >= rb else rb)) / 2.0
                pts.append((rf, mid))
            return [(r * math.cos(t), r * math.sin(t)) for r, t in pts]

        def sketch_gear(comp, x, z, y0, y1, teeth, m, bore, name, look, phase=0.0):
            planes = comp.constructionPlanes
            ym = (y0 + y1) / 2.0
            want = SY * ym * MM
            pl = None
            for sgn in (1, -1):
                pi_ = planes.createInput()
                pi_.setByOffset(comp.xZConstructionPlane, adsk.core.ValueInput.createByReal(sgn * want))
                cand = planes.add(pi_)
                if abs(cand.geometry.origin.y - want) < 1e-6:
                    pl = cand
                    break
                cand.deleteMe()
            if pl is None:
                raise Exception('plane placement')
            pl.isLightBulbOn = False
            sk = comp.sketches.add(pl)
            sk.name = name + ' profile'
            sk.isComputeDeferred = True
            spts = []
            for (u, v) in involute_outline(teeth, m, phase=phase):
                spts.append(sk.sketchPoints.add(sk.modelToSketchSpace(P3(x + u, ym, z + v))))
            lines = sk.sketchCurves.sketchLines
            for i in range(len(spts)):
                lines.addByTwoPoints(spts[i], spts[(i + 1) % len(spts)])
            sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P3(x, ym, z)), bore * MM)
            sk.isComputeDeferred = False
            best, area = None, 0
            for i in range(sk.profiles.count):
                pr = sk.profiles.item(i)
                ar = pr.areaProperties().area
                if ar > area:
                    best, area = pr, ar
            ei = comp.features.extrudeFeatures.createInput(best, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            ei.setSymmetricExtent(adsk.core.ValueInput.createByReal((y1 - y0) * MM), True)
            f = comp.features.extrudeFeatures.add(ei)
            b = f.bodies.item(0)
            b.name = name
            apply(b, look)
            printed.append((comp.name, b))
            sk.isVisible = False
            return b

        # ---------------- looks (material + colour) ----------------
        LOOKS = {
            'frame':   (['PLA', 'Plastic'], ['Plastic - Matte', 'Plastic'], (55, 58, 62)),
            'moving':  (['PLA', 'Plastic'], ['Plastic - Matte', 'Plastic'], (235, 110, 25)),
            'housing': (['PLA', 'Plastic'], ['Plastic - Matte', 'Plastic'], (205, 207, 210)),
            'tpu':     (['Rubber', 'Plastic'], ['Rubber', 'Plastic - Matte', 'Plastic'], (200, 30, 40)),
            'wood':    (['Plywood', 'Wood', 'Oak'], ['Wood - Plywood', 'Plywood', 'Oak', 'Wood'], None),
            'spool':   (['ABS Plastic', 'ABS', 'Plastic'], ['Plastic - Glossy', 'Plastic'], (22, 22, 24)),
            'filament': (['PLA', 'Plastic'], ['Plastic - Glossy', 'Plastic'], (0, 145, 190)),
            'steel':   (['Steel', 'Stainless Steel'], ['Steel - Satin', 'Steel - Polished', 'Steel'], None),
            'alu':     (['Aluminum 6061', 'Aluminum'], ['Aluminum - Satin', 'Aluminum - Polished', 'Aluminum'], None),
            'motor':   (['Steel'], ['Plastic - Matte', 'Plastic'], (30, 30, 32)),
            'black':   (['ABS Plastic', 'ABS', 'Plastic'], ['Plastic - Matte', 'Plastic'], (25, 25, 27)),
            'pcb':     (['Plastic'], ['Plastic - Glossy', 'Plastic'], (20, 110, 60)),
            'pcbblue': (['Plastic'], ['Plastic - Glossy', 'Plastic'], (25, 70, 160)),
            'screen':  (['Glass', 'Plastic'], ['Plastic - Glossy', 'Plastic'], (10, 12, 18)),
            'red':     (['ABS Plastic', 'Plastic'], ['Plastic - Glossy', 'Plastic'], (210, 25, 25)),
            'yellow':  (['ABS Plastic', 'Plastic'], ['Plastic - Glossy', 'Plastic'], (240, 200, 20)),
            'rubber':  (['Rubber', 'Plastic'], ['Rubber', 'Plastic - Matte', 'Plastic'], (15, 15, 15)),
        }
        mat_index, app_index = {}, {}
        for i in range(app.materialLibraries.count):
            lib = app.materialLibraries.item(i)
            try:
                for j in range(lib.materials.count):
                    mm_ = lib.materials.item(j)
                    mat_index.setdefault(mm_.name.lower(), mm_)
            except Exception:
                pass
            try:
                for j in range(lib.appearances.count):
                    a = lib.appearances.item(j)
                    app_index.setdefault(a.name.lower(), a)
            except Exception:
                pass

        def find(index, cands):
            keys = sorted(index)
            for c in cands:
                if c.lower() in index:
                    return index[c.lower()]
            for c in cands:
                for k in keys:
                    if k.startswith(c.lower()):
                        return index[k]
            for c in cands:
                for k in keys:
                    if c.lower() in k:
                        return index[k]
            return None

        def set_colour(appearance, rgb):
            col = adsk.core.Color.create(rgb[0], rgb[1], rgb[2], 255)
            props = appearance.appearanceProperties
            for pid in ('opaque_albedo', 'generic_diffuse', 'metal_f0', 'layered_diffuse', 'Color'):
                p = None
                try:
                    p = adsk.core.ColorProperty.cast(props.itemById(pid))
                except Exception:
                    p = None
                if not p:
                    try:
                        p = adsk.core.ColorProperty.cast(props.itemByName(pid))
                    except Exception:
                        p = None
                if p:
                    try:
                        p.value = col
                        return True
                    except Exception:
                        pass
            for i in range(props.count):
                p = adsk.core.ColorProperty.cast(props.item(i))
                if p:
                    try:
                        p.value = col
                        return True
                    except Exception:
                        pass
            return False

        looks, colour_fail = {}, []
        for key, (mc, ac, rgb) in LOOKS.items():
            mat = find(mat_index, mc)
            base = find(app_index, ac)
            appear = None
            if base:
                nm = 'Spooler - ' + key
                appear = design.appearances.itemByName(nm)
                if not appear:
                    try:
                        appear = design.appearances.addByCopy(base, nm)
                        if rgb and not set_colour(appear, rgb):
                            colour_fail.append(key)
                    except Exception:
                        appear = base
            looks[key] = (mat, appear)

        def apply(body, key):
            mat, appear = looks.get(key, (None, None))
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

        # ---------------- components ----------------
        state = {'comps': True}
        made = {}
        failed = []
        PRINTED = ('frame', 'moving', 'housing', 'tpu')
        printed = []      # (component, body)
        ghosts = []       # bodies excluded from the clash check

        def make(name, items, grounded=True):
            """items: list of (body name, temp body, look). Returns (occ, comp)."""
            occ, comp = None, root
            if state['comps']:
                try:
                    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                    comp = occ.component
                    comp.name = name
                except Exception:
                    state['comps'] = False
                    occ, comp = None, root
            try:
                if parametric:
                    bf = comp.features.baseFeatures.add()
                    bf.startEdit()
                    for _, tb, _ in items:
                        comp.bRepBodies.add(tb, bf)
                    bf.finishEdit()
                    try:
                        bf.name = name
                    except Exception:
                        pass
                    bodies = [bf.bodies.item(i) for i in range(bf.bodies.count)]
                else:
                    bodies = [comp.bRepBodies.add(tb) for _, tb, _ in items]
                for (bn, _, look), b in zip(items, bodies):
                    try:
                        b.name = bn if occ else '%s - %s' % (name, bn)
                    except Exception:
                        pass
                    if look == 'ghost':
                        apply(b, 'spool')
                        try:
                            b.opacity = 0.15
                        except Exception:
                            pass
                        ghosts.append(b.name)
                    else:
                        apply(b, look)
                    if look in PRINTED:
                        printed.append((name, b))
            except Exception:
                failed.append('%s: %s' % (name, traceback.format_exc().splitlines()[-1]))
            if occ is not None:
                try:
                    occ.isGrounded = grounded
                except Exception:
                    pass
            made[name] = (occ, comp)
            return occ, comp

        def build(name, fn, grounded=True):
            try:
                return make(name, fn(), grounded)
            except Exception:
                failed.append('%s: %s' % (name, traceback.format_exc().splitlines()[-1]))
                return None, None

        # ---------------- derived layout ----------------
        Rm = P['od_max'] / 2.0
        R = P['od'] / 2.0
        h = Rm + P['clear']                       # shaft height (175)
        x_src = Rm + 40.0                         # 190
        x_rcv = x_src + 2 * Rm + P['gap']         # 720
        L = x_rcv + Rm + 40.0                     # base length (910)
        yb0, yb1 = -P['base_d'] / 2, P['base_d'] / 2
        ut = P['up_t']
        y_up = -90.0                              # upright front face
        y_upb = y_up - ut                         # upright back face
        rs = P['shaft_d'] / 2.0
        r_hole = P['hole'] / 2.0
        y_cb = y_up + 11.0                        # back cone flange clears the bearing boss
        cone_k = 18.0 / 30.0                      # cone: r 42 -> 12 over 18 mm
        y_sp0 = y_cb + (42.0 - r_hole) * cone_k   # spool back face (cone contact)
        y_sp1 = y_sp0 + P['w']
        y_c = (y_sp0 + y_sp1) / 2.0               # filament path line
        y_shaft1 = y_sp0 + P['w_max'] + 45.0
        m = P['module']
        cd = m * (P['z_pinion'] + P['z_gear']) / 2.0 + P['backlash']   # centre distance + printing clearance
        z_p = h + Rm + 75.0                       # stage path height (400)
        x_trv = x_rcv - Rm - 15.0                 # traverse x (555)
        z_t = h + 100.0                           # traverse guide eye height (275)
        x_run = x_src + 15
        x_d = x_src + 135
        x_whl = x_src + 235
        x_spl = x_src + 295
        x_exit = x_trv - 22
        by1 = y_c - 20
        g1 = y_upb - 8.0                          # gear front face (clears rear boss)
        g0 = g1 - P['gear_w']

        # ---------------- FRAME ----------------
        def frame():
            it = []
            it.append(('Base plate (9 mm ply)', box(0, L, yb0, yb1, -P['base_t'], 0), 'wood'))
            for nm, xc in (('source', x_src), ('receiving', x_rcv)):
                up = box(xc - P['up_w'] / 2, xc + P['up_w'] / 2, y_upb, y_up, 0, h + 45)
                up = U(up, cy(xc, h, y_upb - 6, y_up + 4, 19.0))               # bearing boss
                up = U(up, box(xc - P['up_w'] / 2, xc + P['up_w'] / 2, y_upb - 45, y_up + 10, 0, 12))  # foot
                for sx in (-1, 1):                                              # rear gussets only (front clears 300 mm spools)
                    up = U(up, obox((xc + sx * 40, y_upb - 15, 30), (0, -0.70710678, 0.70710678), (1, 0, 0), 60, 8, 8))
                up = D(up, cy(xc, h, y_upb - 10, y_up + 10, 11.05))             # 608 pockets / bore
                for dx in (-38, 38):                                            # M4 holes to the base
                    for yy in (y_upb - 35, y_up + 2):
                        up = D(up, cz(xc + dx, yy, -1, 13, 2.2))
                it.append(('Upright - ' + nm, up, 'frame'))
                for k, (a, b) in enumerate(((y_upb - 6, y_upb + 1), (y_up - 3, y_up + 4))):
                    it.append(('608 bearing - %s %d' % (nm, k + 1), tube_y(xc, h, a, b, 11.0, 4.0), 'steel'))
            # stage beam and posts (beam passes over the largest source spool)
            bz0, bz1 = z_p - 70, z_p - 50
            by0, by1 = y_c - 40, y_c - 20
            x_post2 = x_exit
            # beam in three printable segments joined by half-laps with an M4 bolt
            bx0, bx1 = 15.0, x_post2 + 10
            cuts = [bx0 + (bx1 - bx0) * k / 3.0 for k in (1, 2)]
            edges = [bx0] + cuts + [bx1]
            bzm = (bz0 + bz1) / 2
            for k in range(3):
                a, b = edges[k], edges[k + 1]
                seg = box(a - (15 if k > 0 else 0), b + (15 if k < 2 else 0), by0, by1, bz0, bz1)
                if k > 0:   # left lap: keep top half
                    seg = D(seg, box(a - 16, a + 15, by0 - 1, by1 + 1, bz0 - 1, bzm))
                    seg = D(seg, cz(a, (by0 + by1) / 2, bz0 - 1, bz1 + 1, 2.2))
                if k < 2:   # right lap: keep bottom half
                    seg = D(seg, box(b - 15, b + 16, by0 - 1, by1 + 1, bzm, bz1 + 1))
                    seg = D(seg, cz(b, (by0 + by1) / 2, bz0 - 1, bz1 + 1, 2.2))
                for xp in (25.0, x_post2):
                    if a - 15 <= xp <= b + 15:
                        seg = D(seg, cz(xp, (by0 + by1) / 2, bz0 - 1, bz1 + 1, 2.2))
                it.append(('Stage beam %d of 3' % (k + 1), seg, 'frame'))
            # posts in two printable halves with a locating spigot
            for nm, xp in (('left', 25.0), ('right', x_post2)):
                ym_ = (by0 + by1) / 2
                zs = bz0 / 2.0
                lo = box(xp - 10, xp + 10, by0, by1, 0, zs)
                lo = U(lo, box(xp - 25, xp + 25, by0 - 5, by1 + 15, 0, 10), cz(xp, ym_, zs, zs + 15, 5.0))
                for dx in (-18, 18):
                    lo = D(lo, cz(xp + dx, ym_, -1, 11, 2.2))
                up_ = D(box(xp - 10, xp + 10, by0, by1, zs, bz0), cz(xp, ym_, zs - 1, zs + 15.5, 5.15),
                        cz(xp, ym_, bz0 - 20, bz0 + 1, 2.2))
                it.append(('Stage post - %s lower' % nm, lo, 'frame'))
                it.append(('Stage post - %s upper' % nm, up_, 'frame'))
            # pulley brackets on the beam front face
            for nm, xx in (('guide 1', x_d - 45), ('guide 2', x_d + 45), ('exit', x_exit)):
                pb = box(xx - 7, xx + 7, by1, by1 + 6, bz0, z_p - 11.9 + 7)
                pb = D(pb, cy(xx, z_p - 11.9, by1 - 1, by1 + 7, 4.05), cy(xx, bz0 + 10, by1 - 1, by1 + 7, 2.2))
                it.append(('Pulley bracket - ' + nm, pb, 'frame'))
            # traverse towers
            yr0 = y_sp0 - 25.0
            yr1 = y_sp0 + P['w_max'] + 25.0
            zs_t = (z_t + 15) / 2.0
            for nm, (a, b) in (('rear', (yr0 - 15, yr0)), ('front', (yr1, yr1 + 15))):
                ym_ = (a + b) / 2
                lo = box(x_trv - 20, x_trv + 20, a, b, 0, zs_t)
                lo = U(lo, box(x_trv - 30, x_trv + 30, a - 10, b + 10, 0, 10), cz(x_trv, ym_, zs_t, zs_t + 15, 5.0))
                for dx in (-24, 24):
                    lo = D(lo, cz(x_trv + dx, ym_, -1, 11, 2.2))
                up_ = box(x_trv - 20, x_trv + 20, a, b, zs_t, z_t + 15)
                up_ = D(up_, cz(x_trv, ym_, zs_t - 1, zs_t + 15.5, 5.15))
                up_ = D(up_, cy(x_trv, z_t - 30, a - 1, b + 1, 4.1), cy(x_trv, z_t - 10, a - 1, b + 1, 4.1))
                if nm == 'rear':   # motor arm for the belt drive
                    up_ = U(up_, box(x_trv + 20, x_trv + 39, a, b, z_t - 52, z_t - 46))
                else:              # idler axle hole
                    up_ = D(up_, cx(ym_, z_t - 20, x_trv + 9, x_trv + 21, 2.55))
                it.append(('Traverse tower - %s lower' % nm, lo, 'frame'))
                it.append(('Traverse tower - %s upper' % nm, up_, 'frame'))
            xbm = x_trv + 30.0
            mp = box(xbm + 6, xbm + 9, yr0 - 7.5 - 21, yr0 - 7.5 + 21, z_t - 46, z_t + 1)
            mp = D(mp, cx(yr0 - 7.5, z_t - 20, xbm + 5, xbm + 10, 11.5))
            for dy in (-15.5, 15.5):
                for dz in (-15.5, 15.5):
                    mp = D(mp, cx(yr0 - 7.5 + dy, z_t - 20 + dz, xbm + 5, xbm + 10, 1.6))
            it.append(('Traverse motor plate', mp, 'frame'))
            # motor bracket for the winding motor (standoffs clear the 60T gear)
            g_back = g0
            bk_y1 = g_back - 3
            bk_y0 = bk_y1 - 5
            zc = h - cd
            br = box(x_rcv - 48, x_rcv + 48, bk_y0, bk_y1, zc - 28, zc + 28)
            br = D(br, cy(x_rcv, zc, bk_y0 - 1, bk_y1 + 1, 11.5))
            for dx, dz in ((-15.5, -15.5), (15.5, -15.5), (-15.5, 15.5), (15.5, 15.5)):
                br = D(br, cy(x_rcv + dx, zc + dz, bk_y0 - 1, bk_y1 + 1, 1.6))
            for dx in (-40, 40):
                for dz in (-20, 20):
                    br = U(br, D(cy(x_rcv + dx, zc + dz, bk_y1, y_upb, 4.0), cy(x_rcv + dx, zc + dz, bk_y1 - 1, y_upb + 1, 1.6)))
            it.append(('Motor bracket', br, 'frame'))
            # brake servo bracket and hall sensor bracket on the source upright (rear)
            yd0, yd1 = y_upb - 22, y_upb - 8
            sv = box(x_src + 30, x_src + 122, yd0 - 8, y_upb, h - 120, h - 113)
            sv = U(sv, box(x_src + 118, x_src + 122, yd0 - 8, y_upb, h - 120, h - 60))
            it.append(('Brake servo bracket', sv, 'frame'))
            hb = box(x_src - 6, x_src + 6, yd0 - 10, y_upb, h + 48, h + 52)
            it.append(('Hall sensor bracket', hb, 'frame'))
            it.append(('Brake pivot boss', tube_y(x_src - 30, h - 51, yd1, y_upb, 6.0, 3.05), 'frame'))
            return it
        frame_occ, frame_comp = build('Frame', frame, grounded=True)

        # ---------------- SPOOL ROTORS ----------------
        def spool_body(xc):
            ft = P['flange_t']
            r_core = P['core'] / 2
            r_bore = r_hole + 3.5
            fl = []
            for y0, y1 in ((y_sp0, y_sp0 + ft), (y_sp1 - ft, y_sp1)):
                f = tube_y(xc, h, y0, y1, R, r_hole)
                for k in range(6):
                    a = math.radians(30 + 60 * k)
                    f = D(f, cy(xc + 78 * math.cos(a), h + 78 * math.sin(a), y0 - 1, y1 + 1, 11.0))
                for k in range(3):
                    a = math.radians(90 + 120 * k)
                    f = D(f, cy(xc + 92 * math.cos(a), h + 92 * math.sin(a), y0 - 1, y1 + 1, 1.2))
                fl.append(f)
            body = U(fl[0], fl[1])
            yi0, yi1 = y_sp0 + ft - 0.5, y_sp1 - ft + 0.5
            body = U(body, tube_y(xc, h, yi0, yi1, r_core, r_core - 3), tube_y(xc, h, yi0, yi1, r_bore, r_hole))
            mid = (r_bore + r_core - 3) / 2
            ln = (r_core - 3) - r_bore + 2
            for dx, dz, lx, lz in ((mid, 0, ln, 3), (-mid, 0, ln, 3), (0, mid, 3, ln), (0, -mid, 3, ln)):
                body = U(body, box(xc + dx - lx / 2, xc + dx + lx / 2, yi0, yi1, h + dz - lz / 2, h + dz + lz / 2))
            return body

        def wound(xc, rf):
            ft = P['flange_t']
            y0, y1 = y_sp0 + ft, y_sp1 - ft
            w = tube_y(xc, h, y0, y1, rf - 0.6, P['core'] / 2)
            n = int((y1 - y0) / P['fil_d'])
            for i in range(n):
                a = y0 + i * P['fil_d'] + 0.2
                w = U(w, tube_y(xc, h, a, a + 0.9, rf, rf - 1.2))
            return w

        def rotor_common(xc):
            it = []
            it.append(('Shaft 8 mm', cy(xc, h, y_upb - 30, y_shaft1, rs), 'steel'))
            bc = D(cy(xc, h, y_cb, y_cb + 18, 42.0, 12.0), cy(xc, h, y_cb - 1, y_cb + 19, rs + 0.2))
            bc = U(bc, D(cy(xc, h, y_cb - 6, y_cb, 42.0), cy(xc, h, y_cb - 7, y_cb + 1, rs + 0.2)))
            it.append(('Cone hub - back (fixed)', bc, 'moving'))
            y_fb = y_sp1 + (42.0 - r_hole) * cone_k        # front cone big end
            fc = D(cy(xc, h, y_fb, y_fb - 18, 42.0, 12.0), cy(xc, h, y_fb + 1, y_fb - 19, rs + 0.2))
            fc = U(fc, D(cy(xc, h, y_fb, y_fb + 6, 42.0), cy(xc, h, y_fb - 1, y_fb + 7, rs + 0.2)))
            it.append(('Cone hub - front (sliding)', fc, 'moving'))
            it.append(('Clamp spring', tube_y(xc, h, y_fb + 6, y_fb + 26, 7.5, 5.0), 'steel'))
            knob = D(cy(xc, h, y_fb + 26, y_fb + 40, 20.0), cy(xc, h, y_fb + 25, y_fb + 41, rs + 0.2))
            for k in range(8):
                a = 2 * math.pi * k / 8
                knob = D(knob, cy(xc + 21 * math.cos(a), h + 21 * math.sin(a), y_fb + 25, y_fb + 41, 3.0))
            it.append(('Thumb nut', knob, 'moving'))
            return it

        def src_rotor():
            it = rotor_common(x_src)
            yd0, yd1 = y_upb - 22, y_upb - 8
            drum = D(cy(x_src, h, yd0, yd1, P['drum_r']), cy(x_src, h, yd0 - 1, yd1 + 1, rs + 0.2))
            drum = D(drum, cy(x_src, h, yd0 + 3, yd1 - 3, P['drum_r'] - 4))       # web lightening
            drum = U(drum, tube_y(x_src, h, yd0, yd1, 12.0, rs + 0.2))
            it.append(('Brake drum', drum, 'moving'))
            for a in (90, 270):
                ra = math.radians(a)
                it.append(('Hall magnet', cy(x_src + 30 * math.cos(ra), h + 30 * math.sin(ra), yd0 - 2, yd0, 3.0), 'steel'))
            it.append(('Spool (reference)', spool_body(x_src), 'spool'))
            it.append(('Filament - full', wound(x_src, P['fill_src']), 'filament'))
            it.append(('Envelope - 300 mm spool (ghost)', cy(x_src, h, y_sp0, y_sp0 + P['w_max'], Rm), 'ghost'))
            return it
        src_occ, src_comp = build('Source spool rotor', src_rotor, grounded=False)

        def rcv_rotor():
            it = rotor_common(x_rcv)
            it.append(('Spool (reference)', spool_body(x_rcv), 'spool'))
            it.append(('Filament - part wound', wound(x_rcv, P['fill_rcv']), 'filament'))
            it.append(('Envelope - 300 mm spool (ghost)', cy(x_rcv, h, y_sp0, y_sp0 + P['w_max'], Rm), 'ghost'))
            return it
        rcv_occ, rcv_comp = build('Receiving spool rotor', rcv_rotor, grounded=False)
        gear_notes = []

        def add_gear(comp, x, z, teeth, bore, name, hub_r, hub_l=10.0, phase=0.0):
            if comp is None:
                return
            try:
                sketch_gear(comp, x, z, g0, g1, teeth, m, bore, name, 'moving', phase)
                gear_notes.append('%s: involute' % name)
            except Exception:
                gear_notes.append('%s: stand-in (%s)' % (name, traceback.format_exc().splitlines()[-1][:60]))
                make_extra(comp, name, gear_y(x, z, g0, g1, teeth, m, bore))
            try:   # hub with grub-screw boss
                make_extra(comp, name + ' hub', D(cy(x, z, g0 - hub_l, g0, hub_r), cy(x, z, g0 - hub_l - 1, g0 + 1, bore)))
            except Exception:
                pass

        def make_extra(comp, name, tb):
            if parametric:
                bf = comp.features.baseFeatures.add()
                bf.startEdit()
                comp.bRepBodies.add(tb, bf)
                bf.finishEdit()
                b = bf.bodies.item(0)
            else:
                b = comp.bRepBodies.add(tb)
            b.name = name
            apply(b, 'moving')
            printed.append((comp.name, b))
            return b

        # mesh: a gear tooth points down at the pinion, a pinion gap points up at the gear
        add_gear(rcv_comp, x_rcv, h, P['z_gear'], rs + 0.2, 'Gear %dT m%.1f' % (P['z_gear'], m), 14.0, phase=-math.pi / 2)

        # ---------------- DRIVE ----------------
        bk_y0 = g0 - 3 - 5
        zc = h - cd

        def motor():
            body = box(x_rcv - 21, x_rcv + 21, bk_y0 - 40, bk_y0, zc - 21, zc + 21)
            for dx in (-21, 21):
                for dz in (-21, 21):
                    body = D(body, cy(x_rcv + dx, zc + dz, bk_y0 - 41, bk_y0 + 1, 4.0))
            boss = cy(x_rcv, zc, bk_y0, bk_y0 + 2, 11.0)
            plug = box(x_rcv - 8, x_rcv + 8, bk_y0 - 40, bk_y0 - 30, zc - 27, zc - 21)
            return [('NEMA 17 17HS4401 (winding)', D(U(body, boss), cy(x_rcv, zc, bk_y0 - 6, bk_y0 + 3, 2.55)), 'motor'),
                    ('Motor connector', plug, 'black')]
        build('Winding motor', motor, grounded=True)

        def pinion():
            return [('Motor shaft 5 mm', cy(x_rcv, zc, bk_y0, g1 + 2, 2.5), 'steel')]
        pin_occ, pin_comp = build('Drive pinion', pinion, grounded=False)
        add_gear(pin_comp, x_rcv, zc, P['z_pinion'], 2.6, 'Pinion %dT m%.1f' % (P['z_pinion'], m), 9.0, 4.0,
                 phase=math.pi / 2 - math.pi / P['z_pinion'])

        # ---------------- BRAKE ----------------
        yd0, yd1 = y_upb - 22, y_upb - 8
        piv = (x_src - 30, h - 51)

        def lever():
            lv = box(x_src - 38, x_src + 100, yd0, yd1, h - 56, h - 46)
            lv = U(lv, cy(piv[0], piv[1], yd0, yd1, 7.0))
            lv = D(lv, cy(piv[0], piv[1], yd0 - 1, yd1 + 1, 3.2))
            pin = cy(piv[0], piv[1], yd0 - 2, y_upb, 3.0)
            pad = box(x_src - 10, x_src + 10, yd0 + 1, yd1 - 1, h - 46, h - 40.2)
            return [('Brake lever (4:1)', lv, 'moving'), ('Pivot pin 6 mm', pin, 'steel'),
                    ('Brake pad (TPU)', pad, 'tpu')]
        lev_occ, lev_comp = build('Brake lever', lever, grounded=False)

        def servo():
            sx0, sx1 = x_src + 70, x_src + 110.7
            sb = box(sx0, sx1, yd0 - 4, yd0 + 16, h - 113, h - 70)
            ears = box(sx0 - 7, sx1 + 7, yd0 - 4, yd0 + 16, h - 82, h - 79.5)
            horn_c = (sx1 - 10, h - 70)
            spline = cz(horn_c[0], yd0 + 6, h - 70, h - 66, 3.0)
            horn = box(horn_c[0] - 3, horn_c[0] + 3, yd0 + 3, yd0 + 9, h - 66, h - 63)
            link = cz(x_src + 95, yd0 + 6, h - 66, h - 56, 1.5)
            return [('MG996R servo', U(sb, ears), 'black'), ('Servo spline', spline, 'steel'),
                    ('Servo horn', horn, 'housing'), ('Push rod', link, 'steel')]
        build('Brake servo', servo, grounded=True)

        def hall():
            return [('A3144 hall sensor', box(x_src - 2.5, x_src + 2.5, yd0 - 6, yd0 - 4, h + 27, h + 31), 'black'),
                    ('Hall board', box(x_src - 6, x_src + 6, yd0 - 8, yd0 - 6, h + 25, h + 48), 'pcb')]
        build('Hall sensor', hall, grounded=True)

        # ---------------- STAGES ----------------

        def runout():
            hs = box(x_run - 17, x_run + 17, y_c - 11, y_c + 11, z_p - 11, z_p + 11)
            hs = D(hs, cx(y_c, z_p, x_run - 18, x_run + 18, 1.2))
            fit = cx(y_c, z_p, x_run - 25, x_run - 17, 3.5)
            mount = D(box(x_run - 10, x_run + 10, by1, y_c - 11, z_p - 70, z_p - 11), cy(x_run, z_p - 60, by1 - 1, y_c, 2.2))
            return [('Runout sensor module', hs, 'black'), ('PTFE push fitting', fit, 'housing'),
                    ('Runout mount', mount, 'frame')]
        build('Runout sensor', runout, grounded=True)

        def pulley(xc, zc_, y):
            pl = U(cy(xc, zc_, y - 4, y - 3, 13.0), cy(xc, zc_, y - 3, y + 3, 11.0), cy(xc, zc_, y + 3, y + 4, 13.0))
            return D(pl, cy(xc, zc_, y - 5, y + 5, 4.0))

        def dancer_fixed():
            it = []
            for k, xx in enumerate((x_d - 45, x_d + 45)):
                it.append(('Guide pulley %d' % (k + 1), pulley(xx, z_p - 11.9, y_c), 'moving'))
                it.append(('Pulley axle %d' % (k + 1), cy(xx, z_p - 11.9, by1 + 6, y_c + 5, 4.0), 'steel'))
            pot = D(cy(x_d + 70, z_p - 33, by1 - 12, by1, 8.5), cy(x_d + 70, z_p - 33, by1 - 13, by1 + 1, 3.05))
            it.append(('10k potentiometer', pot, 'pcbblue'))
            dm = box(x_d + 60, x_d + 80, by1, y_c - 8, z_p - 70, z_p - 25)
            dm = D(dm, cy(x_d + 70, z_p - 60, by1 - 1, y_c, 2.2))
            dm = D(dm, cy(x_d + 70, z_p - 33, by1 - 1, y_c - 7, 3.1))
            it.append(('Dancer mount', dm, 'frame'))
            it.append(('Spring anchor', U(box(x_d + 15, x_d + 25, by1, y_c - 6, z_p - 20, z_p - 16), box(x_d + 15, x_d + 25, by1, by1 + 4, z_p - 70, z_p - 16)), 'frame'))
            return it
        build('Dancer frame', dancer_fixed, grounded=True)

        def dancer_arm():
            piv_d = (x_d + 70, z_p - 33)
            zc_d = z_p - 45 + 11.9                 # filament wraps under the dancer pulley
            arm = box(x_d, x_d + 70, y_c - 8, y_c - 4, zc_d - 4, zc_d + 4)
            arm = U(arm, cy(piv_d[0], piv_d[1], y_c - 8, y_c - 4, 6.0), cy(x_d, zc_d, y_c - 8, y_c - 4, 6.0))
            arm = D(arm, cy(piv_d[0], piv_d[1], y_c - 9, y_c - 3, 3.2), cy(x_d, zc_d, y_c - 9, y_c - 3, 4.05))
            pin = cy(piv_d[0], piv_d[1], by1 - 3, y_c - 4, 3.0)
            pl = pulley(x_d, zc_d, y_c)
            axle = cy(x_d, zc_d, y_c - 8, y_c + 5, 4.0)
            spring = cyl((x_d + 20, y_c - 6, zc_d + 4), (x_d + 20, y_c - 6, z_p - 20), 2.0)
            return [('Dancer arm', arm, 'moving'), ('Dancer pivot pin', pin, 'steel'),
                    ('Dancer pulley', pl, 'moving'), ('Dancer pulley axle', axle, 'steel'),
                    ('Tension spring', spring, 'steel')]
        dan_occ, dan_comp = build('Dancer arm', dancer_arm, grounded=False)

        def wheel_block():
            blk = box(x_whl - 20, x_whl + 20, by1, y_c + 14, z_p - 70, z_p - 2)
            blk = D(blk, cy(x_whl, z_p - 60, by1 - 1, y_c + 15, 2.2))
            blk = D(blk, box(x_whl - 21, x_whl + 21, y_c - 7, y_c + 7, z_p - 25, z_p - 1))
            blk = D(blk, cy(x_whl, z_p - 5.4, by1 - 1, y_c + 15, 8.05))
            it = [('Wheel housing', blk, 'frame')]
            for k, yy in enumerate((y_c - 13, y_c + 8)):
                it.append(('625 bearing %d' % (k + 1), tube_y(x_whl, z_p - 5.4, yy, yy + 5, 8.0, 2.5), 'steel'))
            it.append(('AS5600 board', box(x_whl - 11.5, x_whl + 11.5, by1 - 7, by1 - 5, z_p - 17, z_p + 6), 'pcbblue'))
            it.append(('AS5600 chip', box(x_whl - 2.5, x_whl + 2.5, by1 - 5, by1 - 4, z_p - 8, z_p - 3), 'black'))
            idl_arm = D(box(x_whl - 28, x_whl + 15, y_c + 8, y_c + 12, z_p + 9, z_p + 15),
                        cy(x_whl, z_p + 11.9, y_c + 7, y_c + 13, 4.05))
            it.append(('Idler arm', idl_arm, 'moving'))
            it.append(('Idler 608', tube_y(x_whl, z_p + 11.9, y_c - 3.5, y_c + 3.5, 11.0, 4.0), 'steel'))
            it.append(('Idler axle', cy(x_whl, z_p + 11.9, y_c - 3.5, y_c + 12, 4.0), 'steel'))
            it.append(('Idler spring seat', box(x_whl - 28, x_whl - 20, y_c + 6, y_c + 14, z_p - 14, z_p - 10), 'frame'))
            it.append(('Idler spring', cz(x_whl - 24, y_c + 10, z_p - 10, z_p + 9, 2.5), 'steel'))
            return it
        build('Measuring wheel block', wheel_block, grounded=True)

        def wheel():
            shaft = cy(x_whl, z_p - 5.4, by1 - 2, y_c + 13, 2.5)
            g = cy(x_whl, z_p - 5.4, y_c - 5.5, y_c + 5.5, 4.5)
            g = D(g, cy(x_whl, z_p - 5.4, y_c - 0.8, y_c + 0.8, 4.6, None))
            g = U(g, cy(x_whl, z_p - 5.4, y_c - 0.8, y_c + 0.8, 3.8))
            g = D(g, cy(x_whl, z_p - 5.4, y_c - 6, y_c + 6, 2.55))
            mag = cy(x_whl, z_p - 5.4, by1 - 4, by1 - 2, 3.0)
            return [('Wheel shaft 5 mm', shaft, 'steel'), ('MK8 drive gear', g, 'alu'),
                    ('Diametric magnet', mag, 'steel')]
        whl_occ, whl_comp = build('Measuring wheel', wheel, grounded=False)

        def splicer():
            hs = box(x_spl - 35, x_spl + 35, y_c - 18, y_c + 18, z_p - 20, z_p + 20)
            hs = D(hs, box(x_spl - 33, x_spl + 33, y_c - 16, y_c + 16, z_p - 18, z_p + 18))
            hs = D(hs, box(x_spl - 12, x_spl + 12, y_c - 12, y_c + 12, z_p + 17, z_p + 21))
            hs = D(hs, cx(y_c, z_p, x_spl - 36, x_spl + 36, 3.0))
            blk = D(box(x_spl - 10, x_spl + 10, y_c - 10, y_c + 10, z_p - 11, z_p + 7),
                    cx(y_c, z_p, x_spl - 11, x_spl + 11, 2.1),
                    cy(x_spl, z_p - 6.5, y_c - 11, y_c + 11, 3.05), cy(x_spl + 6, z_p + 4, y_c - 11, y_c + 11, 1.55))
            cart = cy(x_spl, z_p - 6.5, y_c - 16, y_c + 10, 3.0)
            therm = cy(x_spl + 6, z_p + 4, y_c - 16, y_c + 10, 1.5)
            ptfe = D(cx(y_c, z_p, x_spl - 33, x_spl + 33, 2.0), cx(y_c, z_p, x_spl - 34, x_spl + 34, 1.0))
            fan = D(box(x_spl - 15, x_spl + 15, y_c - 15, y_c + 15, z_p + 20, z_p + 30), cz(x_spl, y_c, z_p + 19, z_p + 31, 13))
            blade = cz(x_spl, y_c, z_p + 21, z_p + 29, 5)
            tf = box(x_spl - 5, x_spl + 5, y_c + 10, y_c + 13, z_p - 3, z_p + 3)     # on the block face
            mount = D(box(x_spl - 10, x_spl + 10, by1, y_c - 18, z_p - 70, z_p - 20), cy(x_spl, z_p - 60, by1 - 1, y_c, 2.2))
            return [('Splicer housing', hs, 'housing'), ('Heater block', blk, 'alu'),
                    ('Heater cartridge 12V 40W', cart, 'steel'), ('Thermistor 100k', therm, 'yellow'),
                    ('PTFE tube', ptfe, 'housing'), ('Fan 30 mm', fan, 'black'), ('Fan blades', blade, 'black'),
                    ('Thermal fuse 240C', tf, 'red'), ('Splicer mount', mount, 'frame')]
        build('Splicer (stretch goal)', splicer, grounded=True)

        def exit_guide():
            return [('Exit pulley', pulley(x_exit, z_p - 11.9, y_c), 'moving'),
                    ('Exit axle', cy(x_exit, z_p - 11.9, by1 + 6, y_c + 5, 4.0), 'steel')]
        build('Exit guide', exit_guide, grounded=True)

        # ---------------- TRAVERSE ----------------
        yr0 = y_sp0 - 25.0
        yr1 = y_sp0 + P['w_max'] + 25.0
        zb = z_t - 20.0          # belt pulley centre
        xb = x_trv + 30.0        # belt plane (outside the towers)

        def traverse_fixed():
            it = []
            it.append(('Smooth rod 8 mm - lower', cy(x_trv, z_t - 30, yr0 - 15, yr1 + 15, 4.0), 'steel'))
            it.append(('Smooth rod 8 mm - upper', cy(x_trv, z_t - 10, yr0 - 15, yr1 + 15, 4.0), 'steel'))
            mot = box(xb + 9, xb + 32, yr0 - 7.5 - 21, yr0 - 7.5 + 21, zb - 21, zb + 21)
            it.append(('NEMA 17 pancake (traverse)', mot, 'motor'))
            it.append(('Traverse motor shaft', cx(yr0 - 7.5, zb, xb - 6, xb + 9, 2.5), 'steel'))
            it.append(('GT2 20T pulley', D(cx(yr0 - 7.5, zb, xb - 4, xb + 4, 6.4), cx(yr0 - 7.5, zb, xb - 5, xb + 5, 2.55)), 'alu'))
            it.append(('GT2 idler', D(cx(yr1 + 7.5, zb, xb - 4, xb + 4, 6.4), cx(yr1 + 7.5, zb, xb - 5, xb + 5, 2.55)), 'alu'))
            it.append(('Idler axle', cx(yr1 + 7.5, zb, x_trv + 10, xb + 6, 2.5), 'steel'))
            it.append(('GT2 belt - top', box(xb - 3, xb + 3, yr0 - 7.5, yr1 + 7.5, zb + 6.4, zb + 7.8), 'rubber'))
            it.append(('GT2 belt - bottom', box(xb - 3, xb + 3, yr0 - 7.5, yr1 + 7.5, zb - 7.8, zb - 6.4), 'rubber'))
            it.append(('Endstop switch', box(x_trv - 6, x_trv + 6, yr0, yr0 + 6, z_t - 4, z_t + 6), 'black'))
            return it
        build('Traverse frame', traverse_fixed, grounded=True)

        def carriage():
            yc0, yc1 = y_c - 15, y_c + 15
            cb = box(x_trv - 14, x_trv + 14, yc0, yc1, z_t - 42, z_t + 8)
            cb = D(cb, cy(x_trv, z_t - 30, yc0 - 1, yc1 + 1, 7.6), cy(x_trv, z_t - 10, yc0 - 1, yc1 + 1, 7.6))
            cb = U(cb, box(x_trv + 14, xb + 4, yc0 + 5, yc1 - 5, zb + 4, zb + 12))   # belt clamp arm
            cb = D(cb, box(xb - 3.3, xb + 3.3, yc0, yc1, zb + 6.2, zb + 8.0))         # belt slot
            cb = D(cb, cx(y_c, z_t + 2, x_trv - 15, x_trv + 15, 2.0))                # guide eye
            it = [('Traverse carriage', cb, 'moving')]
            for zz, nm in ((z_t - 30, 'lower'), (z_t - 10, 'upper')):
                it.append(('LM8UU - ' + nm, tube_y(x_trv, zz, yc0 + 3, yc1 - 3, 7.5, 4.0), 'steel'))
            it.append(('Guide eye (PTFE)', D(cx(y_c, z_t + 2, x_trv - 16, x_trv + 16, 2.0),
                                              cx(y_c, z_t + 2, x_trv - 17, x_trv + 17, 1.0)), 'housing'))
            return it
        car_occ, car_comp = build('Traverse carriage', carriage, grounded=False)

        # ---------------- ELECTRONICS ENCLOSURE ----------------
        ex0, ex1, ey0, ey1, ez1 = x_rcv - 110, x_rcv + 70, 50.0, 160.0, 60.0

        def electronics():
            it = []
            bx = D(box(ex0, ex1, ey0, ey1, 0, ez1), box(ex0 + 3, ex1 - 3, ey0 + 3, ey1 - 3, 3, ez1 + 1))
            bx = D(bx, box(ex0 + 47, ex0 + 117, ey1 - 4, ey1 + 1, 10, 52))          # screen window
            bx = D(bx, cy(ex1 - 30, 30, ey1 - 4, ey1 + 1, 4))                        # buzzer
            bx = D(bx, box(ex1 - 4, ex1 + 1, ey0 + 20, ey0 + 34, 30, 50))            # rocker switch cut-out
            bx = D(bx, cx(ey0 + 60, 20, ex1 - 4, ex1 + 1, 6))                        # DC jack
            bx = D(bx, cx(ey0 + 85, 20, ex1 - 4, ex1 + 1, 6.05))                     # fuse holder
            for k in range(6):                                                       # vents
                bx = D(bx, box(ex0 - 1, ex0 + 4, ey0 + 15 + k * 12, ey0 + 20 + k * 12, 20, 45))
            it.append(('Enclosure', bx, 'housing'))
            it.append(('Lid', D(box(ex0, ex1, ey0, ey1, ez1, ez1 + 3), cz(ex1 - 30, ey0 + 30, ez1 - 1, ez1 + 4, 11)), 'housing'))
            it.append(('2.8in touchscreen PCB', box(ex0 + 38, ex0 + 124, ey1 - 9, ey1 - 7.4, 4, 54), 'pcb'))
            it.append(('2.8in touchscreen panel', box(ex0 + 45, ex0 + 119, ey1 - 7.4, ey1 - 3.5, 9, 53), 'screen'))
            it.append(('E-stop base', cz(ex1 - 30, ey0 + 30, ez1 + 3, ez1 + 28, 13), 'yellow'))
            it.append(('E-stop mushroom', cz(ex1 - 30, ey0 + 30, ez1 + 28, ez1 + 40, 20), 'red'))
            it.append(('Rocker switch', box(ex1 - 1, ex1 + 4, ey0 + 21, ey0 + 33, 31, 49), 'black'))
            it.append(('DC jack', cx(ey0 + 60, 20, ex1 - 2, ex1 + 8, 5.5), 'black'))
            it.append(('Fuse holder', cx(ey0 + 85, 20, ex1 - 2, ex1 + 14, 6.0), 'black'))
            it.append(('ESP32-S3 DevKitC-1', box(ex0 + 10, ex0 + 73, ey0 + 10, ey0 + 36, 8, 9.6), 'black'))
            it.append(('Perfboard', box(ex0 + 8, ex1 - 8, ey0 + 8, ey0 + 70, 3, 4.6), 'pcb'))
            for k in range(2):
                xx = ex0 + 85 + k * 25
                it.append(('TMC2209 %d' % (k + 1), box(xx, xx + 15, ey0 + 12, ey0 + 32, 8, 9.6), 'pcbblue'))
                it.append(('TMC2209 heatsink %d' % (k + 1), box(xx + 2, xx + 13, ey0 + 15, ey0 + 29, 9.6, 18), 'alu'))
            it.append(('XL4015 buck', box(ex0 + 85, ex0 + 136, ey0 + 40, ey0 + 66, 8, 9.6), 'pcbblue'))
            it.append(('Buck heatsink', box(ex0 + 105, ex0 + 120, ey0 + 45, ey0 + 61, 9.6, 22), 'alu'))
            it.append(('MOSFET module (D4184)', box(ex0 + 140, ex0 + 168, ey0 + 40, ey0 + 56, 8, 9.6), 'pcbblue'))
            it.append(('PCF8574 expander', box(ex0 + 10, ex0 + 36, ey0 + 45, ey0 + 61, 8, 9.6), 'pcbblue'))
            return it
        build('Electronics enclosure', electronics, grounded=True)

        # ---------------- FILAMENT PATH ----------------
        def filament():
            rf = P['fil_d'] / 2
            pts = [(x_src, y_c, h + P['fill_src'] + rf), (x_run - 25, y_c, z_p), (x_d - 45, y_c, z_p),
                   (x_d - 13, y_c, z_p - 45), (x_d + 13, y_c, z_p - 45), (x_d + 45, y_c, z_p),
                   (x_exit, y_c, z_p), (x_trv, y_c, z_t + 2), (x_rcv - 20, y_c, h + P['fill_rcv'] + rf)]
            return [('Filament path', rod_path(pts, rf), 'filament')]
        build('Filament path', filament, grounded=True)

        # ---------------- JOINTS ----------------
        joints_ok, joints_bad = [], []

        def cyl_face(comp, r_mm):
            for bi in range(comp.bRepBodies.count):
                b = comp.bRepBodies.item(bi)
                for fi in range(b.faces.count):
                    f = b.faces.item(fi)
                    if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                        if abs(f.geometry.radius - r_mm * MM) < 1e-4:
                            return f
            return None

        def joint(name, occ, comp, r_mm, kind='rev'):
            if occ is None or frame_occ is None:
                joints_bad.append(name + ' (no component)')
                return
            try:
                f = cyl_face(comp, r_mm)
                if f is None:
                    joints_bad.append(name + ' (axis face not found)')
                    return
                fp = f.createForAssemblyContext(occ)
                geo = adsk.fusion.JointGeometry.createByNonPlanarFace(fp, adsk.fusion.JointKeyPointTypes.MiddleKeyPoint)
                ji = root.asBuiltJoints.createInput(occ, frame_occ, geo)
                if kind == 'rev':
                    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
                else:
                    ji.setAsSliderJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
                j = root.asBuiltJoints.add(ji)
                try:
                    j.name = name
                except Exception:
                    pass
                joints_ok.append(name)
            except Exception:
                joints_bad.append('%s (%s)' % (name, traceback.format_exc().splitlines()[-1][:80]))

        if state['comps']:
            joint('Source spool', src_occ, src_comp, rs)
            joint('Receiving spool', rcv_occ, rcv_comp, rs)
            joint('Pinion', pin_occ, pin_comp, 2.5)
            joint('Brake lever', lev_occ, lev_comp, 3.0)
            joint('Dancer arm', dan_occ, dan_comp, 3.0)
            joint('Measuring wheel', whl_occ, whl_comp, 2.5)
            joint('Traverse carriage', car_occ, car_comp, 4.0, kind='slide')

        # ---------------- PRINT LIST ----------------
        rows, over, total_g = [], [], 0.0
        for cname, b in printed:
            try:
                bb = b.boundingBox
                dims = sorted([(bb.maxPoint.x - bb.minPoint.x) / MM, (bb.maxPoint.y - bb.minPoint.y) / MM,
                               (bb.maxPoint.z - bb.minPoint.z) / MM], reverse=True)
                vol = b.volume / (MM ** 3) / 1000.0          # cm3
                grams = vol * 1.24 * 0.4                     # PLA, ~15% infill + 3 walls (rough)
                total_g += grams
                rows.append((cname, b.name, dims[0], dims[1], dims[2], vol, grams))
                if dims[0] > BED:
                    over.append('%s / %s (%.0f mm)' % (cname, b.name, dims[0]))
            except Exception:
                pass
        csv_note = ''
        try:
            path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'PrintList.csv')
            with open(path, 'w') as fh:
                fh.write('Component,Part,Longest (mm),Middle (mm),Shortest (mm),Solid volume (cm3),Est. PLA (g)\n')
                for r in sorted(rows):
                    fh.write('"%s","%s",%.0f,%.0f,%.0f,%.1f,%.0f\n' % r)
            csv_note = 'Print list saved: ' + path
        except Exception as e:
            csv_note = 'Print list not saved (%s)' % e.__class__.__name__

        # ---------------- CLASH CHECK ----------------
        clashes, clash_err = [], ''
        try:
            ents = adsk.core.ObjectCollection.create()
            for i in range(root.occurrences.count):
                occ = root.occurrences.item(i)
                if occ.component.name == 'Filament path':
                    continue
                for j in range(occ.bRepBodies.count):
                    b = occ.bRepBodies.item(j)
                    if 'ghost' in b.name:
                        continue
                    ents.add(b)
            ii = design.createInterferenceInput(ents)
            try:
                ii.areCoincidentFacesIncluded = False
            except Exception:
                pass
            res = design.analyzeInterference(ii)

            def label(e):
                try:
                    c = e.assemblyContext.component.name if e.assemblyContext else e.parentComponent.name
                except Exception:
                    c = '?'
                return '%s/%s' % (c, e.name)
            for k in range(res.count):
                r = res.item(k)
                try:
                    v = r.interferenceBody.volume * 1000.0
                except Exception:
                    v = 0.0
                clashes.append((v, '%s x %s' % (label(r.entityOne), label(r.entityTwo))))
            clashes.sort(reverse=True)
        except Exception:
            clash_err = traceback.format_exc().splitlines()[-1][:100]

        try:
            app.activeViewport.goHome()
        except Exception:
            pass
        app.activeViewport.fit()

        msg = ['Spooler assembly built: %d components.' % root.occurrences.count]
        msg.append('Spool shaft height %.0f mm, spool centres %.0f mm apart, base %.0f x %.0f mm.' % (h, x_rcv - x_src, L, P['base_d']))
        msg.append('Gear %.0f:1 (%dT/%dT, module %.1f, centre distance %.2f mm). %s' % (
            P['z_gear'] / P['z_pinion'], P['z_pinion'], P['z_gear'], m, cd, '; '.join(gear_notes)))
        msg.append('Printed parts: %d, about %.0f g of PLA in total. %s' % (len(rows), total_g, csv_note))
        msg.append('Over the %.0f mm bed: %s' % (BED, ', '.join(over) if over else 'none'))
        if clash_err:
            msg.append('Clash check failed: ' + clash_err)
        else:
            msg.append('Clashes found: %d' % len(clashes))
            for v, t in clashes[:25]:
                msg.append('  %.1f mm3  %s' % (v, t))
        if not state['comps']:
            msg.append('\nCOMPONENTS BLOCKED: this design is Part intent, so everything went into Bodies and no joints were made.')
        msg.append('\nJoints made: ' + (', '.join(joints_ok) if joints_ok else 'none'))
        if joints_bad:
            msg.append('Joints failed: ' + '; '.join(joints_bad))
        if failed:
            msg.append('\nParts failed: ' + '; '.join(failed))
        if colour_fail:
            msg.append('Colour not set for: ' + ', '.join(colour_fail))
        msg.append('\n' + '; '.join(notes))
        ui.messageBox('\n'.join(msg))
    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
