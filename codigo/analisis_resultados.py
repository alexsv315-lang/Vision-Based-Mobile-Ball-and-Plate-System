"""Metrics and figures for the flat-floor and ramp tests.
Same segmentation as in the lab presentation: sessions split by time resets,
fixed trial windows, 2 s trimmed at both ends, rows without detection removed."""
import sys, json
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

F10, F11, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
B='#2a78d6'; O='#eb6834'; INK='#1b2a41'; MUT='#6b7280'; GRID='#e5e7eb'; RED='#c0392b'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12,'axes.edgecolor':'#9ca3af',
    'axes.labelcolor':INK,'xtick.color':MUT,'ytick.color':MUT,'axes.spines.top':False,'axes.spines.right':False})

def sesiones(p):
    d=pd.read_csv(p,sep=';'); d['ses']=(d.t.diff()<0).cumsum(); return d
def tramo(d,s,a,b,trim=2):
    g=d[(d.ses==s)&(d.t>=a+trim)&(d.t<=b-trim)&d.x_med_cm.notna()].copy()
    g['tr']=g.t-g.t.iloc[0]; g['ex']=g.x_est_cm-g.ref_x_cm; g['ey']=g.y_est_cm-g.ref_y_cm; g['e']=np.hypot(g.ex,g.ey)
    return g
d10=sesiones(F10); d11=sesiones(F11)
print(list(d10.columns))
piso=[tramo(d10,1,270.52,480.71),tramo(d10,1,1027.39,1540.0),tramo(d10,1,1900.88,2144.95)]
rampa=[tramo(d11,0,40.0,144.8),tramo(d11,0,162.0,262.7),tramo(d11,5,4.9,117.3),tramo(d11,5,132.0,208.6)]

def col(g,*names):
    for n in names:
        if n in g.columns: return g[n]
    return None
def met(g):
    m=dict(dur=g.tr.max(),n=len(g),exm=g.ex.abs().mean(),eym=g.ey.abs().mean(),em=g.e.mean(),eM=g.e.max(),
           p2=(g.e<2).mean()*100,p1=(g.e<1).mean()*100)
    tx=col(g,'theta_des_x'); ty=col(g,'theta_des_y')
    if tx is not None:
        m.update(tx_rms=np.sqrt((tx**2).mean()),ty_rms=np.sqrt((ty**2).mean()),
                 tx_max=tx.abs().max(),ty_max=ty.abs().max(),
                 sat=((tx.abs()>=9.9)|(ty.abs()>=9.9)).mean()*100)
    return m
out={}
for name,T in [('piso',piso),('rampa',rampa)]:
    M=[met(g) for g in T]; out[name]=M
    print(name)
    for i,m in enumerate(M,1): print(i,{k:round(float(v),2) for k,v in m.items()})
    print('mean',{k:round(float(np.mean([m[k] for m in M])),2) for k in M[0]})
json.dump(out,open(OUT+'/metricas.json','w'),indent=1,default=float)

def stack(T,fname,ycols,ylabels,lim,title,hline=None,ylab='[cm]'):
    n=len(T); f,ax=plt.subplots(n,1,figsize=(12,1.9*n+0.7),sharey=True)
    for i,(a,g) in enumerate(zip(ax,T),1):
        for c,colr,lab in zip(ycols,[B,O],ylabels):
            a.plot(g.tr,g[c],color=colr,lw=1.0,label=lab)
        if hline is not None: a.axhline(hline,color=RED,lw=1.1,ls='--',label=f'{hline} cm')
        else: a.axhline(0,color=INK,lw=1,ls='--',label='Reference (0)')
        a.set_ylim(*lim); a.grid(axis='y',color=GRID); a.set_ylabel(f'Run {i}\n{ylab}',fontsize=11); a.set_xlim(0,g.tr.max())
    ax[-1].set_xlabel('Time [s]')
    ax[0].legend(loc='lower right',ncol=3,frameon=False,fontsize=11,bbox_to_anchor=(1,1.0))
    f.suptitle(title,x=0.01,ha='left',fontsize=13,color=INK,fontweight='bold'); f.tight_layout(); f.savefig(fname,dpi=170); plt.close(f)

for name,T,tt in [('flat',piso,'flat floor, C path'),('ramp',rampa,'10° ramp (up, down and reverse)')]:
    stack(T,f'{OUT}/Fig_{name}_position.png',['ex','ey'],['X','Y'],(-7,7),f'Ball position relative to the reference — {tt}')
    for g in T: g['zero']=np.nan
    stack(T,f'{OUT}/Fig_{name}_error.png',['e','zero'],['Distance to reference',''],(0,7),f'Distance between the ball and the reference — {tt}',hline=2)
    if 'theta_des_x' in T[0].columns:
        stack(T,f'{OUT}/Fig_{name}_effort.png',['theta_des_x','theta_des_y'],['Tilt X','Tilt Y'],(-11,11),
              f'Control effort: commanded plate tilt — {tt}',ylab='[deg]')
# XY trajectories
f,ax=plt.subplots(1,2,figsize=(11,5.2))
for a,T,tt in [(ax[0],piso,'(a) Flat floor (3 runs)'),(ax[1],rampa,'(b) 10° ramp (4 runs)')]:
    a.add_patch(plt.Rectangle((-10,-10),20,20,fill=False,ec=INK,lw=1.5))
    a.add_patch(plt.Circle((0,0),2,fill=False,ec=RED,ls='--',lw=1.2))
    for g in T: a.plot(g.ex,g.ey,lw=0.5,alpha=0.6)
    a.plot(0,0,'k+',ms=12,mew=2); a.set_aspect('equal'); a.set_xlim(-11,11); a.set_ylim(-11,11)
    a.set_xlabel('X [cm]'); a.set_ylabel('Y [cm]'); a.set_title(tt,fontsize=12,color=INK); a.grid(color=GRID)
f.text(0.5,0.01,'Square: plate edges (20 x 20 cm); dashed circle: 2 cm around the reference',ha='center',fontsize=10.5,color=MUT)
f.tight_layout(rect=(0,0.04,1,1)); f.savefig(f'{OUT}/Fig_xy_trajectories.png',dpi=170); plt.close(f)
