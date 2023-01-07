import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, Pass2
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)


def function1(id_,rev_id,state):
	flag = False
	kub = kubs.kubs(id_,state,pub)
	recv = kubs.kubs(rev_id,state,pub)
	kub.update_state(state)
	recv.update_state(state)
	print(kub.kubs_id)
	g_fsm = Pass2.Pass()
	g_fsm.add_kub(kub)
	g_fsm.add_receiver(recv)

	g_fsm.as_graphviz()
	g_fsm.write_diagram_png()
	print('something before spin')
	flag = g_fsm.spin()

	return flag

rospy.init_node('node',anonymous=False)
start_time = rospy.Time.now()
start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
# rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
dribble = True
count = 0
while True:
	state = None
	rospy.wait_for_service('bsServer',)
	getState = rospy.ServiceProxy('bsServer',bsServer)	
	try:
		state = getState(state)
	except rospy.ServiceException, e:
		print("Error ",e)	
	if state:
		print('lasknfcjscnajnstate',state.stateB.homePos)

		flag = function1(1,2,state.stateB)
		if flag:
			break
if not flag:
	rospy.spin()
