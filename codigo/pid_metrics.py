import pandas as pd, numpy as np, glob, os, json, sys
U='datos_motores/'  # folder with the wheel-speed CSV files
F={'PD_01':'PD_01.csv','PD_03_P1':'PD_03_P1.csv','PD_03_P2':'PD_03_P2.csv','PD_03_P3':'PD_03_P3.csv',
 'PD2_04_1':'PD2_04_1.csv','PD2_04_2':'PD2_04_2.csv','PD2_04_3':'PD2_04_3.csv',
 'PD2_05_1':'PD2_05_1.csv','PD2_05_2':'PD2_05_2.csv','PD2_05_3':'PD2_05_3.csv',
 'PD2_10':'PD2_10.csv','PD_AGRESIVO':'PD_AGRESIVO.csv','PID_suave':'PID_integral_suave.csv',
 'PID_suave_rampa':'PID_integral_suave_en_rampa.csv'}
cols=['t','refR','wR','uR','refL','wL','uL','x','y','th','v','w','ed','eth']
def load(k):
    d=pd.read_csv(U+F[k],header=None,names=cols)
    for c in ['refR','wR','uR','refL','wL','uL']: d[c]=d[c].abs()
    return d
R={}
for k in F:
    d=load(k); r={'dur':d.t.iloc[-1],'fs':len(d)/d.t.iloc[-1]}
    W0=d.refR.iloc[5]
    for s in 'RL':
        w=d['w'+s]; tr=d.t[(w>=0.9*W0)].iloc[0] if (w>=0.9*W0).any() else np.nan
        r['rise_'+s]=tr
        r['os_'+s]=(w[d.t<4].max()/W0-1)*100
    cr=d[(d.v>=0.149)&(d.t>3)]
    r['n_cruise']=len(cr)
    for s in 'RL':
        e=cr['ref'+s]-cr['w'+s]
        r['ess_'+s]=e.mean(); r['erms_'+s]=np.sqrt((e**2).mean()); r['eabs%_'+s]=(e.abs()/cr['ref'+s]).mean()*100
        r['u_'+s]=cr['u'+s].mean(); r['ustd_'+s]=cr['u'+s].std()
    r['dLR']=(cr.wR-cr.wL).mean()
    allu=pd.concat([d.uR,d.uL]); r['u_rms']=np.sqrt((allu**2).mean()); r['u_max']=allu.max(); r['sat%']=(allu>=99).mean()*100
    # end of straight: first time turning command starts
    iturn=d.index[d.w.abs()>0.01]
    i0=iturn[0]-1 if len(iturn) else len(d)-1
    r['x_end']=d.x.iloc[i0]; r['y_end']=d.y.iloc[i0]; r['th_end_deg']=np.degrees(d.th.iloc[i0])
    r['ed_end']=d.ed.iloc[i0]
    r['th_final_deg']=np.degrees(d.th.iloc[-1]); r['eth_final_deg']=np.degrees(d.eth.iloc[-1])
    r['x_final']=d.x.iloc[-1]; r['y_final']=d.y.iloc[-1]; r['ed_final']=d.ed.iloc[-1]
    R[k]=r
T=pd.DataFrame(R).T
pd.set_option('display.width',250)
print(T[['dur','fs','rise_R','rise_L','os_R','os_L','eabs%_R','eabs%_L','ess_R','ess_L','dLR']].astype(float).round(3).to_string())
print(T[['u_R','u_L','ustd_R','ustd_L','u_rms','u_max','sat%','x_end','y_end','th_end_deg','ed_end','x_final','y_final','th_final_deg','eth_final_deg']].astype(float).round(3).to_string())
T.to_csv('metrics.csv')
