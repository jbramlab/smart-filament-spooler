import math
g=9.81
# --- shaft & bearings (worst case: 300 mm spool, ~3.5 kg total, widest 110 mm) ---
d=8e-3; E=200e9; I=math.pi*d**4/64; EI=E*I
W=3.5*g; T=5.0; F=math.hypot(W,T)
a=0.0747   # front bearing to spool centre for a 110 mm wide spool (from CAD)
for L in (0.015,0.035):
    R1=F*a/L; R2=F*(a+L)/L; M=F*a
    s=32*M/(math.pi*d**3)
    defl=F*a**3/(3*EI)+F*a**2*L/(3*EI)   # overhang beam on two simple supports
    print("bearing spacing %2.0f mm: bearing loads %.0f/%.0f N, shaft stress %.0f MPa, spool deflection %.2f mm, tilt from 0.015 mm bearing play %.2f mm"
          %(L*1e3,R1,R2,s/1e6,defl*1e3, 2*0.015e-3/L*a*1e3))
# --- PLA gear teeth (Lewis) ---
m=1.5e-3; b=8e-3; Y17=0.303
for name,tau in (("running 5 N, full spool",5*0.095/(4*0.9)),("motor pull-out 0.40 N m",0.40)):
    Wt=tau/(m*17/2); s=Wt/(b*m*Y17)
    print("pinion %s: tooth force %.1f N, bending %.1f MPa, safety factor %.1f vs 20 MPa printed PLA"%(name,Wt,s/1e6,20e6/s))
# --- motor speed / torque ---
v=0.5; rmin,rmax=0.061,0.095
print("spool speed at 0.5 m/s: %.0f-%.0f rpm, motor %.0f-%.0f rpm; torque at full receiving spool %.3f N m"%(
    v/rmax*60/(2*math.pi),v/rmin*60/(2*math.pi),4*v/rmax*60/(2*math.pi),4*v/rmin*60/(2*math.pi),5*rmax/(4*0.9)))
print("top speed (3.3 mm/s traverse limit = 1.886 rev/s): spool 113 rpm, motor 453 rpm, line %.2f-%.2f m/s"%(1.886*2*math.pi*rmin,1.886*2*math.pi*rmax))
print("job time 1 kg PLA (335 m) at 0.5 m/s: %.1f min"%(335/0.5/60))
# --- measuring wheel ---
r=5.375e-3; c=2*math.pi*r
print("wheel: %.1f mm per turn, %.1f um per AS5600 count; 0.05 mm radius error = %.2f%% = %.1f m over 335 m; after 1 m calibration to +-1 mm: +-%.2f m"%(c*1e3,c/4096*1e6,0.05/5.375*100,335*0.05/5.375,335*0.001))
# --- traverse ---
print("traverse: GT2 20T = 40 mm/rev, 1/16 microstep = %.1f um/step, 3.3 mm/s = %.0f steps/s"%(40/3200*1e3,3.3/(40/3200)))
# --- dancer ---
k_p=670; arm_p=0.070; arm_s=0.050
print("dancer: spring at 50 mm arm k = %.0f N/m, force at set point %.1f N; pot resolution (12-bit, 270 deg) %.3f mm = %.3f N tension"%(
    k_p*(arm_p/arm_s)**2, 2*5*arm_p/arm_s, arm_p*math.radians(270/4096)*1e3, k_p*arm_p*math.radians(270/4096)/2))
# --- splicer heat-up ---
vol=(20*20*18 - math.pi*3.05**2*20 - math.pi*1.55**2*20 - math.pi*2.1**2*20)*1e-9
mblk=2700*vol; Q=mblk*900*(210-20)
print("heater block %.0f g: %.1f kJ to 210 C -> %.0f s at 40 W with no losses (expect ~1.5-2x with losses)"%(mblk*1e3,Q/1e3,Q/40))
