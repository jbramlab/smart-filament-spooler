# SpoolerRecord.py - records the spooler running as image frames.
# Drives the Pinion joint (the motion links turn the gear, both spools and the
# measuring wheel), sweeps the traverse carriage and lets the dancer arm float.
# Frames are saved to Videos\Spooler Animation\<view>\ (three camera views).

import adsk.core, adsk.fusion, traceback, math, os

OUT = os.path.join(os.path.expanduser('~'), 'Videos', 'Spooler Animation')
W, H = 1280, 720
SPOOL_DEG_PER_FRAME = 3.0      # receiving spool turn per frame at full speed
PINION_RATIO = 68.0 / 17.0
TRAVERSE_MM = 28.0
DANCER_DEG = 3.0


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        vp = app.activeViewport

        J = root.asBuiltJoints.itemByName
        pin, trv, dan = J('Pinion'), J('Traverse carriage'), J('Dancer arm')
        if not (pin and trv and dan):
            raise Exception('Run SpoolerAssembly and SpoolerMotion first (joints missing)')

        P3 = adsk.core.Point3D.create
        home = vp.camera
        ext0 = home.viewExtents
        hdir = home.target.vectorTo(home.eye)

        def cam(target, d, factor):
            c = vp.camera
            c.isSmoothTransition = False
            v = adsk.core.Vector3D.create(*d) if d else hdir.copy()
            v.normalize()
            v.scaleBy(250.0)
            t = P3(*target)
            e = t.copy()
            e.translateBy(v)
            c.target = t
            c.eye = e
            c.upVector = adsk.core.Vector3D.create(0, 0, 1)
            c.isFitView = False
            c.viewExtents = ext0 * factor
            vp.camera = c
            adsk.doEvents()

        # see-through motor bracket and motor for the gear close-up; hide joint icons
        see_through = []
        for i in range(root.occurrences.count):
            occ = root.occurrences.item(i)
            for k in range(occ.bRepBodies.count):
                b = occ.bRepBodies.item(k)
                if b.name in ('Motor bracket', 'NEMA 17 17HS4401 (winding)', 'Motor connector'):
                    see_through.append((b, b.opacity))
        try:
            joints_lb = root.isJointsFolderLightBulbOn
            root.isJointsFolderLightBulbOn = False
        except Exception:
            joints_lb = None

        views = [
            ('1 overview', (45.5, 0.0, 20.0), None, 0.72, 120, False),
            ('2 gears rear', (72.0, 11.4, 14.5), (0.45, 1.0, 0.35), 0.22, 90, True),
            ('3 stage front', (42.0, 3.7, 36.0), (0.35, -1.0, 0.45), 0.30, 90, False),
        ]
        total = sum(v[4] for v in views)

        def speed(f):            # 0..1 motor speed profile across the whole recording
            t = f / float(total - 1)
            if t < 0.08:
                return t / 0.08
            if t > 0.90:
                return max(0.0, (1.0 - t) / 0.10)
            return 1.0

        spool = 0.0
        f = 0
        saved = 0
        for name, tgt, d, factor, n, ghost in views:
            folder = os.path.join(OUT, name)
            os.makedirs(folder, exist_ok=True)
            for old in os.listdir(folder):
                if old.endswith('.png'):
                    os.remove(os.path.join(folder, old))
            for b, op in see_through:
                b.opacity = 0.25 if ghost else op
            cam(tgt, d, factor)
            for i in range(n):
                spool += SPOOL_DEG_PER_FRAME * speed(f)
                t = f / float(total)
                pin.jointMotion.rotationValue = math.radians(spool * PINION_RATIO)
                trv.jointMotion.slideValue = TRAVERSE_MM * math.sin(2 * math.pi * 1.5 * t) / 10.0
                dan.jointMotion.rotationValue = math.radians(DANCER_DEG * math.sin(2 * math.pi * 4.0 * t) * speed(f))
                adsk.doEvents()
                vp.refresh()
                if vp.saveAsImageFile(os.path.join(folder, '%04d.png' % i), W, H):
                    saved += 1
                f += 1

        # back to the start
        pin.jointMotion.rotationValue = 0
        trv.jointMotion.slideValue = 0
        dan.jointMotion.rotationValue = 0
        for b, op in see_through:
            b.opacity = op
        if joints_lb is not None:
            root.isJointsFolderLightBulbOn = joints_lb
        vp.camera = home
        adsk.doEvents()
        ui.messageBox('Recorded %d of %d frames to\n%s' % (saved, total, OUT))
    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
