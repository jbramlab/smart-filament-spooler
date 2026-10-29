# SpoolerMotion.py - adds Motion Links to the Smart Filament Spooler assembly
# so the parts move together like the real machine:
#   pinion (motor) -> receiving spool (4:1 gears, opposite direction)
#   receiving spool -> source spool (same filament speed, radii 72 / 95 mm)
#   source spool -> measuring wheel (wheel radius 4.5 mm)
# After running, drive the Pinion joint (Motion Study or Drive Joints) and the rest follows.
# It then checks the link directions and that the gears still mesh without clashing.

import adsk.core, adsk.fusion, traceback, math

R_SRC, R_RCV, R_WHL = 95.0, 72.0, 4.5      # filament radii in the model (mm)
Z_PIN, Z_GEAR = 17, 68


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        out = []

        def J(name):
            j = root.asBuiltJoints.itemByName(name)
            if j is None:
                raise Exception('Joint not found: ' + name)
            return j

        pin, rcv, src, whl = J('Pinion'), J('Receiving spool'), J('Source spool'), J('Measuring wheel')
        allj = [J(n) for n in ('Pinion', 'Receiving spool', 'Source spool', 'Measuring wheel',
                               'Brake lever', 'Dancer arm', 'Traverse carriage')]

        # remove links from an earlier run
        for k in range(root.motionLinks.count - 1, -1, -1):
            root.motionLinks.item(k).deleteMe()

        def link(name, j1, j2, v1, v2):
            li = root.motionLinks.createInput(j1, j2)
            li.valueOne = adsk.core.ValueInput.createByString(v1)
            li.valueTwo = adsk.core.ValueInput.createByString(v2)
            ml = root.motionLinks.add(li)
            try:
                ml.name = name
            except Exception:
                pass
            return ml

        links = [
            (link('Gears 17T-68T', pin, rcv, '360 deg', '%.4f deg' % (360.0 * Z_PIN / Z_GEAR)), pin, rcv, -1),
            (link('Filament speed', rcv, src, '360 deg', '%.4f deg' % (360.0 * R_RCV / R_SRC)), rcv, src, +1),
            (link('Measuring wheel', src, whl, '360 deg', '%.4f deg' % (360.0 * R_SRC / R_WHL)), src, whl, +1),
        ]

        def occ_of(j):
            return j.occurrenceOne

        def world_angle(j):
            m = occ_of(j).transform2
            return math.atan2(m.getCell(0, 2), m.getCell(0, 0))     # rotation about world Y

        def set_rot(j, a):
            j.jointMotion.rotationValue = a
            adsk.doEvents()

        # check each link turns the follower the right way; flip it if not
        for ml, a, b, want in links:
            for attempt in range(2):
                for j in allj:
                    try:
                        if j.jointMotion.jointType == adsk.fusion.JointTypes.SliderJointType:
                            j.jointMotion.slideValue = 0
                        else:
                            j.jointMotion.rotationValue = 0
                    except Exception:
                        pass
                set_rot(a, 0.2)
                wa, wb = world_angle(a), world_angle(b)
                ok = (wa * wb * want) > 0
                if ok or attempt == 1:
                    break
                ml.isReversed = not ml.isReversed
            out.append('%s: %s (driver %.1f deg, follower %.1f deg)' % (
                ml.name, 'OK' if ok else 'CHECK DIRECTION', math.degrees(wa), math.degrees(wb)))

        # mesh check at a few pinion angles
        gear_bodies = []
        for occ in (occ_of(pin), occ_of(rcv)):
            for i in range(occ.bRepBodies.count):
                b = occ.bRepBodies.item(i)
                if b.name.startswith('Pinion ') or b.name.startswith('Gear '):
                    gear_bodies.append(b)
        clash_txt = []
        for deg in (0, 7, 13, 29, 61, 100):
            set_rot(pin, math.radians(deg))
            ents = adsk.core.ObjectCollection.create()
            for b in gear_bodies:
                ents.add(b)
            ii = design.createInterferenceInput(ents)
            res = design.analyzeInterference(ii)
            vol = 0.0
            for k in range(res.count):
                try:
                    vol += res.item(k).interferenceBody.volume * 1000.0
                except Exception:
                    pass
            clash_txt.append('%d deg: %s' % (deg, 'clear' if res.count == 0 else '%.2f mm3' % vol))
        out.append('Gear mesh while turning (%d gear bodies): %s' % (len(gear_bodies), ', '.join(clash_txt)))

        # back to the start position
        for j in allj:
            try:
                if j.jointMotion.jointType == adsk.fusion.JointTypes.SliderJointType:
                    j.jointMotion.slideValue = 0
                else:
                    j.jointMotion.rotationValue = 0
            except Exception:
                pass

        # hide the ghost 300 mm spools so the motion is easy to see
        hidden = 0
        for i in range(root.occurrences.count):
            occ = root.occurrences.item(i)
            for k in range(occ.bRepBodies.count):
                b = occ.bRepBodies.item(k)
                if 'ghost' in b.name:
                    b.isLightBulbOn = False
                    hidden += 1
        out.append('Ghost spools hidden: %d' % hidden)
        out.append('Motion links: %d' % root.motionLinks.count)
        ui.messageBox('\n'.join(out))
    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
