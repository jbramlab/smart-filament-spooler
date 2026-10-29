from PIL import Image, ImageDraw, ImageFont
import os
SRC = "/mnt/user-data/uploads/Spooler Animation"
segs = [
 ("1 overview", 120, "Smart Filament Spooler: winding cycle",
  "Motor drives the receiving spool through 17T/68T gears (4:1); the source spool unwinds at the same filament speed"),
 ("2 gears rear", 90, "Drive close-up (motor and bracket shown see-through)",
  "17-tooth pinion on the NEMA 17 meshing with the 68-tooth spool gear"),
 ("3 stage front", 90, "Filament path",
  "Guide pulleys, dancer arm (tension), measuring wheel, splicer, then the traverse guide lays filament across the spool"),
]
bold = ImageFont.truetype("/usr/share/fonts/opentype/inter/Inter-SemiBold.otf", 30)
reg = ImageFont.truetype("/usr/share/fonts/opentype/inter/Inter-Regular.otf", 21)
small = ImageFont.truetype("/usr/share/fonts/opentype/inter/Inter-Regular.otf", 17)
n = 0
total = sum(s[1] for s in segs)
for name, cnt, title, sub in segs:
    for i in range(cnt):
        im = Image.open(os.path.join(SRC, name, "%04d.png" % i)).convert("RGBA")
        ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)
        d.rectangle([0, 0, im.width, 92], fill=(20, 22, 26, 205))
        d.text((28, 14), title, font=bold, fill=(255, 255, 255, 255))
        d.text((28, 56), sub, font=reg, fill=(235, 110, 25, 255))
        d.rectangle([0, im.height - 34, im.width, im.height], fill=(20, 22, 26, 160))
        d.text((28, im.height - 27), "Fusion motion study render  |  traverse movement sped up for visibility",
               font=small, fill=(220, 220, 220, 255))
        # progress bar
        d.rectangle([0, 92, int(im.width * (n + 1) / total), 96], fill=(235, 110, 25, 255))
        Image.alpha_composite(im, ov).convert("RGB").save("frames/%04d.png" % n)
        n += 1
print(n)
