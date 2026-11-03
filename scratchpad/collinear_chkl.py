import threading, yaml, math, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from moveit_msgs.srv import GetPositionIK, GetStateValidity, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState
ARM=['j1','j2','j3','j4','j5','j6']; FRAME='head_link'
P=yaml.safe_load(open('config/approach_poses.yaml'))['poses']
names=['+CL','+HL','+KL']
rclpy.init(); n=Node('col'); threading.Thread(target=rclpy.spin,args=(n,),daemon=True).start()
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
# current tips + orientations
tips=[]; oris=[]; qs=[]
for nm in names:
    q=[float(v) for v in P[nm]]; t,o=fkpose(q); tips.append(t); oris.append(o); qs.append(q)
tips=np.array(tips)
# best-fit line: mean + principal direction (PCA)
c=tips.mean(0); u,s,vt=np.linalg.svd(tips-c); d=vt[0]
proj=[c+np.dot(t-c,d)*d for t in tips]
print('best-fit line dir',[round(x,3) for x in d])
out={}
for i,nm in enumerate(names):
    move=np.linalg.norm(proj[i]-tips[i])*1000
    q=solve(proj[i], oris[i], qs[i])
    ok = q is not None and valid(q) and math.degrees(q[2])>=0
    st='OK' if ok else 'FAIL'; jj=(' j3=%.0f'%math.degrees(q[2])) if q else ''; print('%s: move %.1f mm  IK %s%s'%(nm,move,st,jj))
    if ok: out[nm]=[float(v) for v in q]
import json; json.dump(out, open('scratchpad/collinear_poses.json','w'))
print('solved', len(out),'/3 -> scratchpad/collinear_poses.json')
rclpy.shutdown()
