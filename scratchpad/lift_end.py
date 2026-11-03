#!/usr/bin/env python3
"""Lift the last N points of a glide off the head along the tool -Z axis until
collision-free, re-solving IK in-branch (seed = the point itself). Ramp the lift
0 -> found, so it stays smooth. Overwrites the file. Collision-checked."""
import os, sys, math, threading, time
import yaml, rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from std_msgs.msg import Header
REPO='/home/farrukh/Desktop/Hairoom/v2'; sys.path.insert(0, REPO+'/scripts')
from run_landmarks import Runner
J=['j1','j2','j3','j4','j5','j6']
FILE=os.environ['FILE']; N=int(os.environ.get('N','18')); WHICH=os.environ.get('WHICH','end')

def fk(node,jv):
    cli=node.create_client(GetPositionFK,'/compute_fk'); cli.wait_for_service(timeout_sec=5)
    js=JointState(); js.name=list(J); js.position=[float(x) for x in jv]
    r=GetPositionFK.Request(); r.header=Header(frame_id='head_link'); r.fk_link_names=['tcp_tool']
    rs=RobotState(); rs.joint_state=js; r.robot_state=rs
    f=cli.call_async(r); t0=time.time()
    while not f.done() and time.time()-t0<5: time.sleep(0.02)
    p=f.result().pose_stamped[0].pose
    return [p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],[p.position.x,p.position.y,p.position.z]

def main():
    d=yaml.safe_load(open(REPO+'/'+FILE)); jn=d['joint_names']; idx=[jn.index(j) for j in J]
    P=d['points']; n=len(P)
    rclpy.init(); node=Runner(); threading.Thread(target=rclpy.spin,args=(node,),daemon=True).start()
    node.sv.wait_for_service(timeout_sec=10); node.ik.wait_for_service(timeout_sec=10)
    t0=time.time()
    while node.js is None and time.time()-t0<8: time.sleep(0.05)
    rng=range(n-N,n) if WHICH=='end' else range(0,N)
    fixed=0
    for k,i in enumerate(rng):
        jv=[P[i]['positions'][m] for m in idx]
        if node._is_valid(dict(zip(J,jv)))[0]:
            continue
        ori,tip=fk(node,jv)
        x,y,z,w=ori; zx=2*(x*z+w*y); zy=2*(y*z-w*x); zz=1-2*(x*x+y*y)
        got=False
        for so_mm in range(1,15):
            so=so_mm/1000.0
            ps=PoseStamped(); ps.header.frame_id='head_link'
            ps.pose.position.x=tip[0]-so*zx; ps.pose.position.y=tip[1]-so*zy; ps.pose.position.z=tip[2]-so*zz
            ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=ori
            sol=node._ik_once(ps, dict(zip(J,jv)))
            if sol and node._is_valid(sol)[0]:
                for m,j in enumerate(J): P[i]['positions'][idx[m]]=sol[j]
                fixed+=1; got=True
                print(f'  idx {i}: lifted {so_mm}mm -> clear'); break
        if not got:
            print(f'  idx {i}: could NOT clear up to 14mm — ABORT'); return 1
    # full recheck
    bad=sum(0 if node._is_valid(dict(zip(J,[p['positions'][m] for m in idx])))[0] else 1 for p in P)
    if bad: print(f'still {bad} collisions — ABORT'); return 1
    yaml.safe_dump(d, open(REPO+'/'+FILE,'w'), default_flow_style=None, sort_keys=False)
    print(f'lifted {fixed} pts, all collision-free, saved {FILE}')
    return 0

if __name__=='__main__':
    rc=main(); sys.stdout.flush(); os._exit(rc)
