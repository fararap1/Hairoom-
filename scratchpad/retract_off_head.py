#!/usr/bin/env python3
"""Cartesian retract: move the tool tip straight out along tool -Z from the
arm's CURRENT (possibly grazing) pose, re-solving IK each mm, until the pose is
collision-free with margin. Writes a short retract path (routes/paths/
retract_off_head.yaml) to be sent DIRECTLY to the controller (no MoveIt plan,
since planning refuses a colliding start). Retract only moves AWAY from the head."""
import os, sys, math, threading, time
import yaml, rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from std_msgs.msg import Header
REPO='/home/farrukh/Desktop/Hairoom/v2'; sys.path.insert(0, REPO+'/scripts')
from run_landmarks import Runner, robot_deg
J=['j1','j2','j3','j4','j5','j6']
OUT='routes/paths/retract_off_head.yaml'
MAX_MM=int(os.environ.get('MAX_MM','15'))
CLEAR_MARGIN=int(os.environ.get('CLEAR_MM','3'))  # keep going this many mm past first clear

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
    rclpy.init(); n=Runner(); threading.Thread(target=rclpy.spin,args=(n,),daemon=True).start()
    n.sv.wait_for_service(timeout_sec=10); n.ik.wait_for_service(timeout_sec=10)
    t0=time.time()
    while n.js is None and time.time()-t0<8: time.sleep(0.05)
    cur=[math.radians(x) for x in robot_deg()]
    ori,tip=fk(n,cur)
    x,y,z,w=ori; zx=2*(x*z+w*y); zy=2*(y*z-w*x); zz=1-2*(x*x+y*y)
    pts=[{'positions':list(cur),'time_from_start':0.0}]
    seed=dict(zip(J,cur)); clear_at=None
    for so_mm in range(1,MAX_MM+1):
        so=so_mm/1000.0
        ps=PoseStamped(); ps.header.frame_id='head_link'
        ps.pose.position.x=tip[0]-so*zx; ps.pose.position.y=tip[1]-so*zy; ps.pose.position.z=tip[2]-so*zz
        ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=ori
        sol=n._ik_once(ps, seed)
        if sol is None:
            print(f'  {so_mm}mm: IK fail'); continue
        jv=[sol[j] for j in J]
        valid=n._is_valid(sol)[0]
        pts.append({'positions':jv,'time_from_start':round(so_mm*0.2,4)})
        seed=sol
        if valid and clear_at is None:
            clear_at=so_mm; print(f'  clear at {so_mm}mm')
        if clear_at is not None and so_mm>=clear_at+CLEAR_MARGIN:
            break
    if clear_at is None:
        print('  never cleared — ABORT'); return 1
    doc={'from':'grazingD','to':'clear','joint_names':list(J),'points':pts}
    yaml.safe_dump(doc,open(REPO+'/'+OUT,'w'),default_flow_style=None,sort_keys=False)
    print(f'retract {len(pts)} pts, cleared at {clear_at}mm -> {OUT}')
    return 0

if __name__=='__main__':
    rc=main(); sys.stdout.flush(); os._exit(rc)
