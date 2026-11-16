#!/usr/bin/python3
"""Attach obstacle memory to an existing mapping launch without resetting it."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from process_supervisor import ManagedChildren

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--owner-pid',type=int,required=True)
    p.add_argument('--report-prefix',default='obstacle_memory_session')
    p.add_argument('--detach',action='store_true')
    p.add_argument('--restore-state-file',type=Path,
                   help='Restore an exported checkpoint from this exact LIO session')
    p.add_argument('--body-frame',default='base_link_3d',
                   help='Physical observation frame (use base_link for legacy launches)')
    args=p.parse_args()
    if Path(args.report_prefix).name!=args.report_prefix:p.error('Invalid report prefix')
    log=ROOT/'reports'/f'{args.report_prefix}.log'
    if args.detach:
        with log.open('w') as stream:
            proc=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),
                *[v for v in sys.argv[1:] if v!='--detach']],stdin=subprocess.DEVNULL,
                stdout=stream,stderr=subprocess.STDOUT,cwd=ROOT,start_new_session=True)
        print(json.dumps({'supervisor_pid':proc.pid,'log':str(log)}));return
    owner=Path(f'/proc/{args.owner_pid}')
    if b'go2_lio_navigation.launch.py' not in (owner/'cmdline').read_bytes().split(b'\0'):
        raise RuntimeError('Unexpected mapping owner')
    managed=ManagedChildren(args.owner_pid)
    env=os.environ.copy()
    original=dict(v.split('=',1) for v in (owner/'environ').read_text().split('\0') if '=' in v)
    for key in ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','CYCLONEDDS_URI'):
        if key in original:env[key]=original[key]
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    report={'owner_pid':args.owner_pid,'owner_identity':managed.owner_identity,'supervisor_pid':os.getpid()}
    target=ROOT/'reports'/f'{args.report_prefix}.json'
    try:
        argv=['/usr/bin/python3',str(ROOT/'ros2_ws/install/go2_lio_navigation/lib/'
                            'go2_lio_navigation/obstacle_memory.py'),'--ros-args',
                            '-p','body_frame:='+args.body_frame]
        if args.restore_state_file:
            argv+=['-p','restore_state_file:='+str(args.restore_state_file.resolve(strict=True))]
        child=managed.start(argv,cwd=ROOT,env=env)
        report['memory_pid']=child.pid
        target.write_text(json.dumps(report,indent=2))
        while managed.owner_alive():
            if child.poll() is not None:raise RuntimeError('Memory node exited')
            time.sleep(.2)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        report['error']=str(exc)
        raise
    finally:
        managed.close();report['stopped']=True;target.write_text(json.dumps(report,indent=2))


if __name__ == '__main__':main()
