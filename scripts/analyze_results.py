#!/usr/bin/env python3
from pathlib import Path
import argparse, json, math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

try:
    from pyulog import ULog
except Exception:
    ULog = None


def rms(x):
    a=np.asarray(x,dtype=float)
    a=a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a*a))) if len(a) else float('nan')


def read_ulog_metrics(run_dir: Path):
    out={"px4_torque_rms":float('nan'),"actuator_sat_pct":float('nan')}
    if ULog is None: return out
    ulgs=sorted(run_dir.glob('*.ulg'), key=lambda p:p.stat().st_mtime)
    if not ulgs: return out
    try:
        u=ULog(str(ulgs[-1]))
        try:
            d=u.get_dataset('vehicle_torque_setpoint').data
            xyz=[]
            for i in range(3):
                k=f'xyz[{i}]'
                if k in d: xyz.append(np.asarray(d[k],float))
            if len(xyz)==3:
                n=min(map(len,xyz))
                mag=np.sqrt(xyz[0][:n]**2+xyz[1][:n]**2+xyz[2][:n]**2)
                out['px4_torque_rms']=rms(mag)
        except Exception:
            pass
        try:
            d=u.get_dataset('actuator_motors').data
            controls=[]
            for i in range(4):
                k=f'control[{i}]'
                if k in d: controls.append(np.asarray(d[k],float))
            if len(controls)==4:
                n=min(map(len,controls)); A=np.column_stack([a[:n] for a in controls])
                valid=np.all(np.isfinite(A),axis=1)
                A=A[valid]
                if len(A): out['actuator_sat_pct']=float(100*np.mean(np.any((A>0.98)|(A<0.02),axis=1)))
        except Exception:
            pass
    except Exception:
        pass
    return out


def analyze(root: Path):
    rows=[]
    for d in sorted(root.iterdir()):
        csv=d/'controller.csv'
        if not csv.exists(): continue
        df=pd.read_csv(csv)
        use=df[(df.flight_t>=6.0) & (df.flight_t <= max(6.0, df.flight_t.max()-1.0))].copy()
        if len(use)<10: use=df.copy()
        pos=np.sqrt((use.x-use.xd)**2+(use.y-use.yd)**2+(use.z-use.zd)**2)
        vel=np.sqrt((use.vx-use.vxd)**2+(use.vy-use.vyd)**2+(use.vz-use.vzd)**2)
        rate=np.sqrt((use.wx-use.wspx)**2+(use.wy-use.wspy)**2+(use.wz-use.wspz)**2)
        tau=np.sqrt(use.taux_norm**2+use.tauy_norm**2+use.tauz_norm**2)
        m={
            'run':d.name,'mode':str(use['mode'].iloc[0]),'scenario':str(use['scenario'].iloc[0]),
            'pos_rmse_m':rms(pos),'pos_max_m':float(np.nanmax(pos)),
            'vel_rmse_mps':rms(vel),'att_rmse_deg':rms(use.att_err_deg),
            'att_max_deg':float(np.nanmax(use.att_err_deg)),
            'rate_tracking_rmse_rads':rms(rate),
            'direct_tau_norm_rms':rms(tau),
            'thrust_norm_mean':float(np.nanmean(use.thrust_norm)),
        }
        m.update(read_ulog_metrics(d)); rows.append(m)
    if not rows: raise SystemExit('No controller.csv files found')
    out=pd.DataFrame(rows).sort_values(['scenario','mode'])
    out.to_csv(root/'summary.csv',index=False)
    (root/'summary.json').write_text(json.dumps(rows,indent=2,allow_nan=True))

    plt.figure(figsize=(8,5))
    for mode in ['rate','torque']:
        s=out[out['mode']==mode]
        plt.plot(s.scenario,s.pos_rmse_m,marker='o',label=mode)
    plt.ylabel('Position RMSE [m]'); plt.xlabel('Scenario'); plt.xticks(rotation=25,ha='right')
    plt.grid(True); plt.legend(); plt.tight_layout(); plt.savefig(root/'position_rmse.png',dpi=180); plt.close()

    plt.figure(figsize=(8,5))
    for mode in ['rate','torque']:
        s=out[out['mode']==mode]
        plt.plot(s.scenario,s.att_rmse_deg,marker='o',label=mode)
    plt.ylabel('SO(3) attitude RMSE [deg]'); plt.xlabel('Scenario'); plt.xticks(rotation=25,ha='right')
    plt.grid(True); plt.legend(); plt.tight_layout(); plt.savefig(root/'attitude_rmse.png',dpi=180); plt.close()

    print(out.to_string(index=False))

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--root',default='results'); args=ap.parse_args()
    analyze(Path(args.root))
