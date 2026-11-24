import pandas as pd, numpy as np, matplotlib, sys
matplotlib.use('Agg'); import matplotlib.pyplot as plt
exec(open('pid_metrics.py').read().split("R={}")[0])
LANG=sys.argv[1]; OUT=sys.argv[2]
ES=LANG=='es'
L=lambda en,es: es if ES else en
B='#2a78d6'; O='#eb6834'; INK='#1b2a41'; MUT='#6b7280'; GRID='#e5e7eb'; RED='#c0392b'; GRN='#1e8449'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12,'axes.spines.top':False,'axes.spines.right':False,'axes.edgecolor':'#9ca3af'})

# ---------- 1. trajectory and wheel loop of the final controller (PD-03, run 1)
d=load('PD_03_P1')
f,ax=plt.subplots(3,1,figsize=(12,9.2),sharex=True)
ph=[(0,34.5,L('Forward 3 m','Avance 3 m')),(34.5,53,L('Turn 180°','Giro 180°')),(53,d.t.max(),L('Return to origin','Regreso al origen'))]
for a in ax:
    for i,(a0,a1,t) in enumerate(ph): a.axvspan(a0,a1,color=['#eef3f9','#fef3c7','#eef3f9'][i],lw=0)
for a0,a1,t in ph: ax[0].text((a0+a1)/2,3.35,t,ha='center',fontsize=11,color=INK)
ax[0].plot(d.t,d.x,color=B,lw=2,label=L('Position x [m]','Posición x [m]'))
a2=ax[0].twinx(); a2.plot(d.t,np.degrees(d.th),color=O,lw=2,label=L('Heading θ [deg]','Orientación θ [°]')); a2.set_ylabel('θ [°]',color=O); a2.set_ylim(-200,20); a2.spines['top'].set_visible(False)
ax[0].set_ylabel('x [m]',color=B); ax[0].set_ylim(-0.1,3.6)
h1,l1=ax[0].get_legend_handles_labels(); h2,l2=a2.get_legend_handles_labels(); ax[0].legend(h1+h2,l1+l2,loc='center left',frameon=False,fontsize=11)
ax[0].set_title(L('(a) Robot trajectory (odometry)','(a) Trayectoria del robot (odometría)'),loc='left',fontsize=12.5,color=INK)
ax[1].plot(d.t,d.wR,color=B,lw=0.6,alpha=0.8,label=L('Right wheel (measured)','Rueda derecha (medida)'))
ax[1].plot(d.t,d.wL,color=O,lw=0.6,alpha=0.7,label=L('Left wheel (measured)','Rueda izquierda (medida)'))
ax[1].plot(d.t,d.refR,color=INK,lw=1.6,ls='--',label=L('Reference','Referencia'))
ax[1].set_ylabel(L('Wheel speed\n[rad/s]','Vel. rueda\n[rad/s]')); ax[1].legend(loc='upper center',ncol=3,frameon=False,fontsize=10.5); ax[1].set_ylim(0,4.2)
ax[1].set_title(L('(b) Wheel-speed tracking (PD controller)','(b) Seguimiento de velocidad de las ruedas (PD)'),loc='left',fontsize=12.5,color=INK)
ax[2].plot(d.t,d.uR,color=B,lw=0.8,label=L('Right','Derecha')); ax[2].plot(d.t,d.uL,color=O,lw=0.8,label=L('Left','Izquierda'))
ax[2].set_ylabel(L('PWM duty\n[%]','PWM\n[%]')); ax[2].set_xlabel(L('Time [s]','Tiempo [s]')); ax[2].legend(loc='upper center',ncol=2,frameon=False,fontsize=10.5); ax[2].set_ylim(0,45)
ax[2].set_title(L('(c) Control effort','(c) Esfuerzo de control'),loc='left',fontsize=12.5,color=INK)
for a in ax: a.grid(axis='y',color=GRID)
ax[2].set_xlim(0,d.t.max())
f.tight_layout(); f.savefig(f'{OUT}/base_trajectory_{LANG}.png',dpi=170); plt.close(f)

# ---------- 2. gain comparison
T=pd.read_csv('metrics.csv',index_col=0)
G=[('PD-01','0.60 / 0.60',['PD_01']),('PD-03','0.80 / 0.80',['PD_03_P1','PD_03_P2','PD_03_P3']),('PD2-04','0.70 / 0.80',['PD2_04_1','PD2_04_2','PD2_04_3']),
   ('PD2-05','0.80 / 0.70',['PD2_05_1','PD2_05_2','PD2_05_3']),('PD2-10','2.0 / 0.90',['PD2_10']),(L('PD aggr.','PD agres.'),'1.10 / 0.95',['PD_AGRESIVO']),(L('PID soft','PID suave'),'0.80 / 0.80',['PID_suave'])]
lab=[g.replace(' ','\n') for g,k,_ in G]
head=[T.loc[ks].th_end_deg.abs().mean() for *_,ks in G]; hsd=[T.loc[ks].th_end_deg.abs().std() if len(ks)>1 else 0 for *_,ks in G]
err=[((T.loc[ks]['eabs%_R']+T.loc[ks]['eabs%_L'])/2).mean() for *_,ks in G]
rip=[((T.loc[ks].ustd_R+T.loc[ks].ustd_L)/2).mean() for *_,ks in G]
bcol=['#9ca3af']*7; bcol[1]=GRN
f,ax=plt.subplots(1,3,figsize=(15,5.0))
x=np.arange(7)
for a,v,sd,t,yl,fmt in [(ax[0],err,None,L('(a) Mean wheel-speed error','(a) Error medio de velocidad'),'[%]','{:.2f}'),
                         (ax[1],head,hsd,L('(b) Heading deviation after 3 m','(b) Desvío de rumbo tras 3 m'),'[°]','{:.2f}'),
                         (ax[2],rip,None,L('(c) PWM ripple (std. dev.)','(c) Rizado del PWM (desv. estándar)'),'[%]','{:.2f}')]:
    a.bar(x,v,color=bcol,yerr=sd,capsize=4 if sd else 0)
    for i,val in enumerate(v): a.text(i,val*1.02+(0.01 if sd is None else hsd[i]),fmt.format(val),ha='center',fontsize=9.5)
    a.set_xticks(x); a.set_xticklabels(lab,fontsize=9.5); a.set_ylabel(yl); a.set_title(t,loc='left',fontsize=12,color=INK); a.grid(axis='y',color=GRID)
ax[0].set_ylim(0,6.5); ax[0].axhline(7.4,color=RED,ls='--')
f.text(0.5,0.005,L('Gains of each test are listed in the comparison table. Green: selected controller. Error bars: s.d. of 3 runs.',
                   'Etiquetas: prueba y Kc derecha / izquierda. Verde: controlador elegido. Barras de error: desv. estándar de 3 repeticiones.'),ha='center',fontsize=9.5,color=MUT)
f.tight_layout(rect=(0,0.05,1,1)); f.savefig(f'{OUT}/base_gains_{LANG}.png',dpi=170); plt.close(f)

# ---------- 3. ramp vs floor (same controller PID soft)
fl=load('PID_suave'); rp=load('PID_suave_rampa')
f,ax=plt.subplots(2,1,figsize=(12,6.6),sharex=True)
for a,s,tt in [(ax[0],'R',L('Right wheel','Rueda derecha')),(ax[1],'L',L('Left wheel','Rueda izquierda'))]:
    for a0,a1 in [(1.5,8.5),(54.5,62)]: a.axvspan(a0,a1,color='#fde2cf',lw=0)
    for a0,a1 in [(12.5,17.5),(64.5,70.5)]: a.axvspan(a0,a1,color='#dbeafe',lw=0)
    a.plot(fl.t,fl['u'+s].rolling(15,center=True).mean(),color=B,lw=1.3,label=L('Flat floor','Piso plano'))
    a.plot(rp.t,rp['u'+s].rolling(15,center=True).mean(),color=O,lw=1.3,label=L('10° ramp','Rampa 10°'))
    a.set_ylabel(f'PWM {tt} [%]',fontsize=11); a.set_ylim(0,45); a.grid(axis='y',color=GRID)
ax[0].legend(loc='lower left',ncol=2,frameon=False)
ax[0].text(5,41,L('climb','subida'),ha='center',color=O,fontsize=11); ax[0].text(58.2,41,L('climb (reverse)','subida (retroceso)'),ha='center',color=O,fontsize=11)
ax[0].text(15,41,L('descent','bajada'),ha='center',color=B,fontsize=11); ax[0].text(67.5,41,L('descent','bajada'),ha='center',color=B,fontsize=11)
ax[1].set_xlabel(L('Time [s]','Tiempo [s]')); ax[1].set_xlim(0,rp.t.max())
f.suptitle(L('Control effort of the wheel loop: flat floor vs 10° ramp (moving average of 15 samples)','Esfuerzo de control de las ruedas: piso vs rampa de 10° (media móvil de 15 muestras)'),x=0.01,ha='left',fontsize=12.5,color=INK)
f.tight_layout(rect=(0,0,1,0.95)); f.savefig(f'{OUT}/base_ramp_{LANG}.png',dpi=170); plt.close(f)

# ---------- 4. appendix: speed and PWM of every tuning (first 20 s)
if not ES:
    ks=[('PD-01','PD_01'),('PD-03 (run 1)','PD_03_P1'),('PD2-04 (run 1)','PD2_04_1'),('PD2-05 (run 1)','PD2_05_1'),('PD2-10','PD2_10'),('PD aggressive','PD_AGRESIVO'),('PID soft','PID_suave'),('PID soft, ramp','PID_suave_rampa')]
    f,ax=plt.subplots(4,2,figsize=(13,12),sharex=True)
    for a,(n,k) in zip(ax.ravel(),ks):
        dd=load(k); s=dd[dd.t<20]
        a.plot(s.t,s.wR,color=B,lw=0.6); a.plot(s.t,s.refR,color=INK,ls='--',lw=1)
        a2=a.twinx(); a2.plot(s.t,s.uR,color=O,lw=0.8); a2.set_ylim(0,110); a2.spines['top'].set_visible(False)
        a.set_ylim(0,4); a.set_title(n,loc='left',fontsize=11.5,color=INK); a.grid(axis='y',color=GRID)
        a.set_ylabel('ω [rad/s]',color=B,fontsize=10); a2.set_ylabel('PWM [%]',color=O,fontsize=10)
    for a in ax[-1]: a.set_xlabel('Time [s]')
    f.suptitle('Right wheel: speed (blue), reference (dashed) and PWM (orange), first 20 s of each test',x=0.01,ha='left',fontsize=12.5,color=INK)
    f.tight_layout(rect=(0,0,1,0.97)); f.savefig(f'{OUT}/base_appendix_en.png',dpi=150); plt.close(f)
print('ok')
