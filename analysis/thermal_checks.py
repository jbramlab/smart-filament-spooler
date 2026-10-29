import math
Ta=20.0
# heater block: 20x20x18 mm aluminium with bores, 40 W 12 V cartridge
A_blk=2*(20*20)+4*(20*18); A_blk*=1e-6
m_blk=0.017; C=m_blk*900+0.006*500+0.4e-3*1000   # block + cartridge + PTFE (J/K)
P=40.0; R=12**2/P; I=12/R
print("cartridge: %.1f ohm, %.2f A at 12 V"%(R,I))
for h,lab in ((15,"fan off (still air)"),(50,"fan on, blowing on block")):
    hA=h*A_blk; Tss=Ta+P/hA; loss=hA*(210-Ta); tau=C/hA
    t=-tau*math.log(1-(210-Ta)*hA/P)
    print("%-26s heat loss at 210 C %.1f W, time to 210 C %.0f s, uncontrolled max %.0f C"%(lab,loss,t,Tss))
# filament heating inside PTFE in the block (20 mm hot length)
k_ptfe=0.25; Rp=math.log(2.0/1.0)/(2*math.pi*k_ptfe)/0.020
C_fil=2.98e-3*0.020*1800; C_ptfe=0.41e-3*1000
print("filament in PTFE: wall resistance %.0f K/W, filament time constant %.1f s (with PTFE heat-up ~%.0f s)"%(Rp,Rp*C_fil,Rp*(C_fil+C_ptfe)))
# thermistor 100k B3950 with 1k pull-up at 3.3 V
def Rt(T): return 100e3*math.exp(3950*(1/(T+273.15)-1/298.15))
for T in (25,180,200,210,220):
    V=3.3*Rt(T)/(Rt(T)+1000); print("thermistor %3d C: %8.0f ohm, %.3f V"%(T,Rt(T),V))
dV=(3.3*Rt(209)/(Rt(209)+1000)-3.3*Rt(211)/(Rt(211)+1000))/2
print("sensitivity at 210 C: %.1f mV/C -> %.2f C per ESP32 ADC count (0.8 mV)"%(dV*1e3,0.8e-3/dV))
# PLA housing around the block (70x36x40 box, block at 210 C, 6 W loss)
A_h=2*(70*36+70*40+36*40)*1e-6
print("splicer housing: steady wall rise about %.0f C above room -> %.0f C (PLA softens ~55-60 C)"%(6/(10*A_h),Ta+6/(10*A_h)))
# brake drum heating: power = tension x line speed
Pb=5*0.5; A_d=2*math.pi*0.04*0.014+2*math.pi*0.04**2
print("brake: %.1f W continuous; drum steady rise about %.0f C (h=20 rotating)"%(Pb,Pb/(20*A_d)))
# winding motor: copper loss 2 phases x I^2 R (17HS4401 ~1.5 ohm/phase)
for I_ in (1.4,1.0,0.8):
    print("motor at %.1f A: copper loss about %.1f W"%(I_,2*I_**2*1.5))
