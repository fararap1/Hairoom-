import threading, yaml, math, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from moveit_msgs.srv import GetPositionIK, GetStateValidity, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState
ARM=['j1','j2','j3','j4','j5','j6']; FRAME='head_link'
CENTER=np.array([0.0025,-0.1872,0.0232])
L={l['name']:l for l in yaml.safe_load(open('config/head_landmarks.yaml'))['landmarks']}
pos=np.array(L['+CL']['position'])
P=yaml.safe_load(open('config/approach_poses.yaml'))['poses']
seed=[float(v) for v in P['+HL']]  # bent-elbow neighbour as seed
rclpy.init(); n=Node('clba'); threading.Thread(target=rclpy.spin,args=(n,),daemon=True).start()
ik=n.create_client(GetPositionIK,'/compute_ik'); ik.wait_for_service(timeout_sec=10)
sv=n.create_client(GetStateValidity,'/check_state_validity'); sv.wait_for_service(timeout_sec=10)
fk=n.create_client(GetPositionFK,'/compute_fk'); fk.wait_for_service(timeout_sec=10)
def call(cl,req):
    ev=threading.Event(); box={}
    cl.call_async(req).add_done_callback(lambda f:(box.__setitem__('r',f.result()),ev.set())); ev.wait(6); return box.get('r')
def fkpose(q):
    r=GetPositionFK.Request(); r.header=Header(frame_id=FRAME); r.fk_link_names=['tcp_tool']
    r.robot_state.joint_state=JointState(name=ARM,position=[float(v) for v in q])
    p=call(fk,r).pose_stamped[0].pose
    return np.array([p.position.x,p.position.y,p.position.z]),[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
def qmul(a,b):
    ax,ay,az,aw=a; bx,by,bz,bw=b
    return [aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz]
def rot(q,v):
    x,y,z,w=q; vx,vy,vz=v
    tx=2*(y*vz-z*vy); ty=2*(z*vx-x*vz); tz=2*(x*vy-y*vx)
    return np.array([vx+w*tx+(y*tz-z*ty),vy+w*ty+(z*tx-x*tz),vz+w*tz+(x*ty-y*tx)])
def aim_from(o_ref, look):
    z=rot(o_ref,(0,0,1)); z=z/np.linalg.norm(z); ld=look/np.linalg.norm(look)
    d=float(np.clip(np.dot(z,ld),-1,1))
    if d>0.99999: return list(o_ref)
    ax=np.cross(z,ld); na=np.linalg.norm(ax)
    if na<1e-9: return list(o_ref)
    ax/=na; ang=math.acos(d); s=math.sin(ang/2)
    return list(np.array(qmul([ax[0]*s,ax[1]*s,ax[2]*s,math.cos(ang/2)],list(o_ref)))/1.0)
def solve(tgt,o,sd):
    ps=PoseStamped(); ps.header.frame_id=FRAME
    ps.pose.position.x,ps.pose.position.y,ps.pose.position.z=[float(v) for v in tgt]
    ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=o
    req=PositionIKRequest(); req.group_name='fairino5_v6_group'; req.ik_link_name='tcp_tool'
    req.pose_stamped=ps; req.timeout.sec=1; req.avoid_collisions=False
    rs=RobotState(); rs.joint_state=JointState(name=ARM,position=[float(v) for v in sd]); req.robot_state=rs
    r=GetPositionIK.Request(); r.ik_request=req; res=call(ik,r)
    if res is None or res.error_code.val!=1: return None
    js=res.solution.joint_state; return [js.position[js.name.index(j)] for j in ARM]
def valid(q):
    r=GetStateValidity.Request(); r.group_name='fairino5_v6_group'
    rs=RobotState(); rs.joint_state=JointState(name=ARM,position=[float(v) for v in q]); r.robot_state=rs
    return call(sv,r).valid
_,o_seed=fkpose(seed)
look=CENTER-pos
o_aim=aim_from(o_seed, look)
best=None
for mm in range(0,60,4):
    tgt=pos - (look/np.linalg.norm(look))*0  # tip AT point; standoff along -look
    tgt=pos + (look/np.linalg.norm(look))*(-mm/1000.0)  # back off away from center
    q=solve(tgt,o_aim,seed)
    if q is None: continue
    j3=math.degrees(q[2])
    if 30<=j3<=160 and valid(q):
        zc=rot(fkpose(q)[1],(0,0,1)); zc=zc/np.linalg.norm(zc)
        aimerr=math.degrees(math.acos(float(np.clip(np.dot(zc,look/np.linalg.norm(look)),-1,1))))
        best=(mm,q,j3,aimerr); break
if best is None:
    print('+CL bent-elbow aimed: NOT found (j3 55..145)')
else:
    mm,q,j3,aimerr=best
    print(f'+CL bent-elbow aimed FOUND: standoff {mm}mm j3={j3:.0f} aim-err={aimerr:.1f}deg j(deg)={[round(math.degrees(x),1) for x in q]}')
rclpy.shutdown()
