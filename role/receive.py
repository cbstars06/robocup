import behavior
from enum import Enum
import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *


class PassReceive(behavior.Behavior):

    class State(Enum):
        main = 1
        Receive = 2
        Pass = 3

    def __init__(self,max_vel = MAX_BOT_SPEED/1.2,min_vel = MIN_BOT_SPEED*1.2):

        



class Receive(behavior.Behavior):

    class State(Enum):
        setup = 1
        in_2_alpha = 2
        outside_circles = 3
        inside_circles = 4
        move_on_circle = 5

    def __init__(self,max_vel = MAX_BOT_SPEED/1.2,min_vel = MIN_BOT_SPEED*1.2):
        
        self.alpha = math.pi/8

        self.d = BOT_BALL_THRESH

        self.radius = self.d/(2*sin(self.alpha))

        self.theta = None
        
        self.circle1 = None

        self.circle2 = None

        self.DISTANCE_THRESH = BOT_BALL_THRESH

        self.ROTATION_FACTOR = ROTATION_FACTOR

        self.behavior_failed = False

        self.behavior_completed = False

        self.FIRST_CALL = True

        self.add_state()

        self.add_state(Receive.State.setup,Pass-Receive.State.main)

        self.add_state(Receive.State.in_2_alpha,Pass-Receive.State.main)

        self.add_state(Receive.State.outside_circles,Pass-Receive.State.main)

        self.add_state(Receive.State.inside_circles,Pass-Receive.State.main)

        self.add_state(Receive.State.move_on_circle, Pass-Receive.State.main)

        
        self.add_transition(Pass-Receive.State.main,Receive.State.setup,lambda: self.receive,'Receive')

        self.add_transition(Receive.State.setup,Receive.State.in_2_alpha, lambda:self.bot_inside_2alpha() ,'In 2 alpha')

        self.add_transition(Receive.State.setup,Receive.State.outside_circles, lambda:self.bot_outside_circles() and not self.bot_inside_2alpha() ,'Outside circles')

        self.add_transition(Receive.State.setup,Receive.State.move_on_circle, lambda:self.bot_on_circle() ,'On circle')

        self.add_transition(Receive.State.outside_circles,Receive.State.move_on_circle, lambda:self.bot_on_circle() ,'On circle')

        self.add_transition(Receive.State.setup,Receive.State.inside_circles, lambda: not self.bot_inside_2alpha() and not self.bot_outside_circles() ,'Inside circles')

        self.add_transition(Receive.State.outside_circles,Receive.State.setup,lambda:self.behavior_failed, 'Outside Circle failed')

        self.add_transition(Receive.State.inside_circles,Receive.State.setup,lambda:self.behavior_failed, 'Inside Circle failed')

        self.add_transition(Receive.State.in_2_alpha,Receive.State.setup,lambda:self.behavior_failed, 'In 2 alpha failed')

        self.add_transition(Receive.State.in_2_alpha,Pass-Receive.State.main,lambda:self.behavior_completed, 'In 2 alpha completed')

        self.add_transition(Receive.State.move_on_circle,Receive.State.setup,lambda:self.behavior_failed, 'Move on Circle failed')

        self.add_transition(Receive.State.move_on_circle,Pass-Receive.State.main,lambda:self.at_target_point() or self.behavior_completed, 'Completed')

        self.add_transition(Receive.State.inside_circles,Pass-Receive.State.main,lambda:self.behavior_completed or self.at_target_point(), 'Inside Circle completed')

