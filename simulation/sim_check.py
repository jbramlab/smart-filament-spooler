import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- parameters (same as spooler_params.m) ----
d=1.75e-3; rho=1240.0; lam=rho*np.pi*d**2/4      # kg/m filament
w=0.061; kp_pack=0.80; rc=0.061                   # wound width, packing density, core radius
L0=1.0/lam                                        # m of filament on a full 1 kg spool
J_shell=0.0025                                    # empty spool + cones + drum (kg m^2), estimate
mu=0.5; r_d=0.040; Fmax=54.0                      # TPU pad on PLA drum, drum radius, max pad force
b_s=1e-4                                          # bearing viscous (N m s)
k_sp=670.0; c_sp=5.0; xf=0.030; x_set=0.015; xmax=0.030  # dancer spring & travel
T_set=k_sp*(xf-x_set)/2                           # 5.0 N
N=4.0; eta=0.9; J_r_shell=0.003                   # gear ratio, efficiency, receiving rotor inertia
tau_srv=0.08                                      # servo lag
v_run=0.5; t_ramp=3.0                             # m/s, s
Kp,Ki,Kd=1500.0,6000.0,250.0
EPS=0.005
def r_of(L): return np.sqrt(rc**2+max(L,0)*d**2/(4*w*kp_pack))

def run(L_start_src, T_end=30.0, dt=2e-4, stop_at=22.0):
    Ls=0.0  # unwound from source
    Lr=0.0
    ws=0.0; x=x_set; Fb=T_set*r_of(L_start_src)/(mu*r_d); I=0.0; eprev=0.0
    out=[]
    t=0.0
    wr_prev=0.0
    while t<T_end:
        # speed profile
        if t<t_ramp: v=v_run*t/t_ramp
        elif t<stop_at: v=v_run
        elif t<stop_at+t_ramp: v=v_run*(1-(t-stop_at)/t_ramp)
        else: v=0.0
        rs=r_of(L_start_src-Ls); rr=r_of(Lr)
        m_f=lam*(L_start_src-Ls)
        Js=J_shell+0.5*m_f*(rc**2+rs**2)
        wr=v/rr
        vr=wr*rr; vs=ws*rs
        xd=(vs-vr)/2
        T=max(0.0,(k_sp*(xf-x)-c_sp*xd)/2)
        # controller (brake): x above set -> loop long -> tension low -> more brake
        e=x-x_set
        I+=e*dt
        D=xd
        Fff=T_set*rs/(mu*r_d)
        Fcmd=np.clip(Fff+Kp*e+Ki*I+Kd*D,0,Fmax)
        if Fcmd in (0,Fmax): I-=e*dt   # anti-windup
        Fb+=(Fcmd-Fb)/tau_srv*dt
        tb=mu*Fb*r_d*np.tanh(ws/EPS)
        wsd=(T*rs-tb-b_s*ws)/Js
        Jr=J_r_shell+0.5*lam*Lr*(rc**2+rr**2)
        alpha_r=(wr-wr_prev)/dt; wr_prev=wr
        tau_m=(T*rr+Jr*alpha_r)/(N*eta)
        out.append((t,v,vs,T,x*1000,Fb,rs*1000,tau_m,ws*60/(2*np.pi),wr*60/(2*np.pi)))
        ws+=wsd*dt; x+=xd*dt; x=min(max(x,0.0),xmax)
        Ls+=vs*dt; Lr+=vr*dt; t+=dt
    return np.array(out)

full=run(L0); low=run(L0*0.03)
for name,a in (("full source (r=95 mm)",full),("nearly empty source",low)):
    run_mask=(a[:,0]>5)&(a[:,0]<21)
    print(name, "start radius %.1f mm"%a[0,6],
          "| tension while running: mean %.2f N, min %.2f, max %.2f"%(a[run_mask,3].mean(),a[run_mask,3].min(),a[run_mask,3].max()),
          "| whole run min/max %.2f/%.2f N"%(a[:,3].min(),a[:,3].max()),
          "| dancer %.1f-%.1f mm"%(a[:,4].min(),a[:,4].max()),
          "| brake force %.1f-%.1f N"%(a[:,5].min(),a[:,5].max()),
          "| peak motor torque %.3f N m"%np.abs(a[:,7]).max())
fig,ax=plt.subplots(4,1,figsize=(9,10),sharex=True)
for a,lab in ((full,"full source spool"),(low,"nearly empty source")):
    ax[0].plot(a[:,0],a[:,1],label="target "+lab); ax[0].plot(a[:,0],a[:,2],'--',label="source pay-out "+lab)
    ax[1].plot(a[:,0],a[:,3],label=lab)
    ax[2].plot(a[:,0],a[:,4],label=lab)
    ax[3].plot(a[:,0],a[:,5],label=lab)
ax[1].axhline(T_set,color='k',lw=0.8,ls=':')
ax[0].set_ylabel("line speed (m/s)"); ax[1].set_ylabel("tension (N)"); ax[2].set_ylabel("dancer (mm)"); ax[3].set_ylabel("brake force (N)")
ax[3].set_xlabel("time (s)")
for a_ in ax: a_.legend(fontsize=7); a_.grid(alpha=.3)
fig.suptitle("Spooler tension loop: start, run at 0.5 m/s, stop (Python check of the Simulink model)")
fig.tight_layout(); fig.savefig("sim_check.png",dpi=110)
