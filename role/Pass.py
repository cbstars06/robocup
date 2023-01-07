from enum import Enum

class Main(object):

    def __init__(self):
    
            print("Main")

class Pass(Main):

    def __init__(self):

        super(Pass,self).__init__()




class Receive(Main):

    def __init__(self):
        super(Receive,self).__init__()

class Goal(Main):


    def __init__(self):
        super(Goal,self).__init__()

class PassReceive(Receive,Pass,Goal,Main):


    def __init__(self):

        super(PassReceive,self).__init__()

        print("MRO:", [x.__name__ for x in PassReceive.__mro__])



pass_receive = PassReceive()