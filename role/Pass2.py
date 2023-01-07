from enum import Enum
import behavior
import _GoToPoint_, _turnAround_
import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *



class Pass(behavior.Behavior):

    class State(Enum):
        setup = 1
        fine_receive = 2
        move_to_point = 3
        kick = 4

    def __init__(self):

        super(Pass, self).__init__()

        self.name = "Pass"

        self.power = 5

        self.alpha = math.pi/12

        self.target_point = None

        self.distance = BOT_RADIUS*4

        self.ball_dist_thresh = BOT_BALL_THRESH

        self.rotate = None

        self.goal = Vector2D(6000,0)

        self.behavior_failed = False

        self.behavior_completed = False

        self.add_state(Pass.State.setup, behavior.Behavior.State.running)
        self.add_state(Pass.State.fine_receive, behavior.Behavior.State.running)
        self.add_state(Pass.State.move_to_point, behavior.Behavior.State.running)
        self.add_state(Pass.State.kick, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start, Pass.State.setup, lambda: True, 'immediately')
        self.add_transition(Pass.State.setup, Pass.State.fine_receive, lambda: self.first_receive(), 'first_receive')
        self.add_transition(Pass.State.setup, Pass.State.move_to_point, lambda: self.at_ball_pos(), 'ball connected')
        self.add_transition(Pass.State.fine_receive, Pass.State.move_to_point, lambda: self.at_ball_pos(), 'ball connected')
        self.add_transition(Pass.State.move_to_point, Pass.State.kick, lambda: self.at_target_point(), 'kick')
        self.add_transition(Pass.State.fine_receive,Pass.State.setup, lambda: self.behavior_failed, 'failed')
        self.add_transition(Pass.State.move_to_point,Pass.State.setup, lambda: self.behavior_failed, 'failed')
        self.add_transition(Pass.State.kick,Pass.State.setup, lambda: self.behavior_failed, 'failed')
        self.add_transition(Pass.State.kick, behavior.Behavior.State.completed, lambda: self.behavior_completed, 'completed')


    def add_kub(self, kub):
        self.kub = kub

    def add_receiver(self, receiver):
        self.receiver = receiver

    def first_receive(self):
        return not kub_has_ball(self.kub.state, self.kub.kubs_id)

    def ball_connected(self):
        return kub_has_ball(self.kub.state, self.kub.kubs_id)

    def at_ball_pos(self):
        return vicinity_points(self.kub.state.ballPos,self.kub.get_pos(),thresh= self.ball_dist_thresh*2) and abs(normalize_angle(self.kub.get_pos().theta-self.theta)) <= ROTATION_FACTOR

    def at_target_point(self):
        return vicinity_points(self.target_point,self.kub.get_pos(),thresh= self.ball_dist_thresh*2) and abs(normalize_angle(self.kub.get_pos().theta-self.theta)) <= ROTATION_FACTOR

    def calculate_theta(self,point):

        rev_pos = Vector2D(self.receiver.get_pos())
        vec_ball_bot = rev_pos - point
        dist = vec_ball_bot.abs(vec_ball_bot)
        
        cos_beta = sin(self.alpha)
        sin_beta = cos(self.alpha)
        point1 = Vector2D()
        point1.x = point.x + (cos_beta*vec_ball_bot.x - sin_beta*vec_ball_bot.y)*dist
        point1.y = point.y + (sin_beta*vec_ball_bot.x + cos_beta*vec_ball_bot.y)*dist


        cos_beta = sin(self.alpha)
        sin_beta = -cos(self.alpha)
        point2 = Vector2D()
        point2.x = point.x + (cos_beta*vec_ball_bot.x - sin_beta*vec_ball_bot.y)*dist
        point2.y = point.y + (sin_beta*vec_ball_bot.x + cos_beta*vec_ball_bot.y)*dist

        if point1.dist(self.goal) < point2.dist(self.goal):
            self.theta = vec_ball_bot.angle() + self.alpha
        else:
            self.theta = vec_ball_bot.angle() - self.alpha



    def on_enter_setup(self):
        self.behavior_failed = False

    def execute_setup(self):
        ball_pos = Vector2D(self.kub.state.ballPos)
        self.calculate_theta(ball_pos)
        pass

    def on_exit_setup(self):
        pass

    def on_enter_fine_receive(self):
        ball_pos = Vector2D(self.kub.state.ballPos)
        self.calculate_theta(ball_pos)
        self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
        _GoToPoint_.init(self.kub, self.target_point,self.theta)
        self.kub.dribble(True)
        self.kub.execute()

    def execute_fine_receive(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.ball_dist_thresh*1.3)
        for gf in generatingfunction:
            self.kub,target = gf
            
            if not vicinity_points(target,self.target_point,thresh=BOT_RADIUS*2):
                self.behavior_failed = True
                break

    def on_exit_fine_receive(self):
        pass

    def on_enter_move_to_point(self):
        rev_pos = Vector2D(self.receiver.get_pos())
        ball_pos = Vector2D(self.kub.state.ballPos)
        vec_ball_bot = rev_pos - ball_pos
        vec_ball_bot = vec_ball_bot*(1/vec_ball_bot.abs(vec_ball_bot))
        self.target_point = ball_pos + vec_ball_bot*self.distance
        self.target_point = getPointBehindTheBall(self.target_point,self.theta)
        self.calculate_theta(self.target_point)
        _GoToPoint_.init(self.kub, self.target_point,self.theta)
        self.kub.dribble(True)
        self.kub.execute()

    def execute_move_to_point(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        generatingfunction = _GoToPoint_.execute(start_time,self.ball_dist_thresh/10)
        for gf in generatingfunction:
            self.kub,target = gf
            if not vicinity_points(self.target_point,target,thresh=BOT_RADIUS*2):
                print 'Failed'
                self.behavior_failed = True
                break

    def on_exit_move_to_point(self):
        self.kub.move(0,0)

    def on_enter_kick(self):
        pass

    def execute_kick(self):

        self.kub.kick(self.power)
        self.kub.execute()
        self.kub.dribble(False)
        self.kub.execute()
        self.behavior_completed = True

    def on_exit_kick(self):
        self.kub.kicked = True
        self.receiver.kicked = False



