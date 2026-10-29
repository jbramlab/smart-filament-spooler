import math
sig=5.67e-8; T0=293.15
# ---- splicer heater block (from the CAD) ----
vol=(20*20*18 - math.pi*3.05**2*20 - math.pi*1.55**2*20 - math.pi*2.1**2*20)*1e-9
m=2700*vol; c=900; A=2*(20*20)+2*(20*18)+2*(20*18); A*=1e-6
P=40.0
def loss(T,h,eps=0.2): return h*A*(T-T0)+eps*sig*A*(T**4-T0**4)
for h,lab in ((12,"fan off"),(60,"fan on")):
    T=T0; t=0; dt=0.05; t210=None
    while t<600:
        T+=(P-loss(T,h))/(m*c)*dt; t+=dt
        if t210 is None and T>=483.15: t210=t
    # steady state with full power
    lo,hi=T0,2000
    for _ in range(60):
        mid=(lo+hi)/2; lo,hi=(mid,hi) if loss(mid,h)<P else (lo,mid)
    print("%s: 20->210 C in %s s, heat loss at 210 C %.1f W, uncontrolled full-power limit %.0f C"%(lab,"%.0f"%t210 if t210 else ">600",loss(483.15,h),lo-273.15))
print("block mass %.0f g, heater current %.2f A (R = %.1f ohm)"%(m*1e3,40/12,12**2/40))
# cool the joint inside the block with the fan: 210 -> 50 C
tau=m*c/(60*A); print("cooling block 210->50 C with fan: about %.0f s (time constant %.0f s)"%(tau*math.log(190/30),tau))
# ---- PLA housing around the block ----
Ah=2*(70*36+70*40+36*40)*1e-6
for Pin in (loss(483.15,12),):
    dTw=Pin/(10*Ah)   # wall to room, ~10 W/m2K outside
    print("housing: %.1f W from block into a %.0f cm2 PLA box -> walls about %.0f C at steady state (PLA softens ~60 C)"%(Pin,Ah*1e4,20+dTw))
# ---- thermistor: 100k NTC, B=3950, 1k pull-up, 3.3 V, 12-bit ----
B=3950
for Tc in (25,200,210,240):
    T=Tc+273.15; R=100e3*math.exp(B*(1/T-1/298.15)); V=3.3*R/(R+1e3)
    dRdT=-B/T**2*R; dVdT=3.3*1e3/(R+1e3)**2*dRdT
    print("thermistor %3d C: %.0f ohm, %.2f V, %.1f mV/K = %.2f K per ADC count"%(Tc,R,V,abs(dVdT)*1e3,0.806e-3/abs(dVdT)))
# ---- brake drum heating (PLA drum) ----
Ad=2*math.pi*0.040*0.014+2*math.pi*0.040**2
for v in (0.5,1.13):
    Pb=5*v; print("brake at %.2f m/s: %.1f W into the PLA drum -> about %.0f C steady"%(v,Pb,20+Pb/(10*Ad)))
# ---- stepper case temperature ----
for I in (1.4,1.0,0.8):
    Pm=2*I**2*1.5   # two phases, ~1.5 ohm per phase (17HS4401)
    print("winding motor at %.1f A: about %.1f W standstill/hold -> case about %.0f C (motor surface ~0.0145 m2, h~10)"%(I,Pm,20+Pm/(10*0.0145)))
