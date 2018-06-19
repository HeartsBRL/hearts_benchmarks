#!/usr/bin/env python
'''
Rooms:
    kitchen, bedroom, living room, dining room, hall
Furniture:
    bed, table lamps, couch, armchairs, chairs, tables, shelves,
Objects:
    mirror, pillows, books, cups, bowls, plates

commands:
    MOVE [forward, backward]
    TURN [left, right]
    GO to the [kitchen, bedroom, living room, dining room, hall (room)]
    ##HERE is the [room]##
    LOOK [Up, Down, Left, Right]
    SEE the [open, closed] door BETWEEN the [room] and the [room]
    SEE the [couch, bed, chair, lamp (furniture)] in the [room]
    SEE the [color] [coke, biscuits (object)] in the [room] on the [furniture]
'''
import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseStamped
from turtlesim.msg import Pose
from math import cos, sin
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rospy.rostime import Duration
from move_base_msgs.msg import MoveBaseActionFeedback
from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState
import std_srvs.srv
from navigation_camera_mgr_example import NavigationCameraMgr

class Controller():
    def __init__(self):
        
        ### Publishers - sends out goal locations, movement/turns, and speech        
        self.pubGoal = rospy.Publisher('hearts/navigation/goal/location', String, queue_size=10)
        self.pub_talk = rospy.Publisher('/hearts/tts', String, queue_size = 10)
        self.pub_pic = rospy.Publisher('/hearts/camera/snapshot', String, queue_size = 10)
        self.pub_head = rospy.Publisher('/head_controller/command',JointTrajectory, queue_size = 10)
        self.pub_move = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)  
        self.pub_dummy = rospy.Publisher('/move_base/feedback', MoveBaseActionFeedback, queue_size = 10)
              
        ### Subscribers - must listen for speech commands, location
        rospy.Subscriber("/hearts/stt", String, self.hearCommand_callback)
        rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.current_pose_callback)
        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)
        
        self.prepare = rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
        self.execute = rospy.ServiceProxy('/roah_rsbb/end_execute', std_srvs.srv.Empty)
    
        self.outfolder = rospy.get_param('output_path')
        
        self.objects = ['banana','apple','lemon','orange','macaroni','weetabix','juice',
        'water','pringles','paracetamol','soda','pasta','soup','toothpaste','salt','gum']

        self.furniture = ['kitchen','dining',
        'arm','cabinet','table','chair','side','coffee',
        'tv','couch','bookshelf','nightstand','night','bed','wardrobe',
        'sofa','bedside','chest','closet','lamp','basket',
        'plant']
        self.rooms = ['kitchen', 'bedroom', 'living_room', 'dining_room', 'hall']
        self.doors = ['bedroom', 'entrance','hall','bathroom']
        self.tbm1_commands_dict = {
            "move": ["forward", "backward"],
            "turn": ["right", "left"],
            "go": self.rooms,
            "look": ["up", "down", "right", "left"],
            "here": [],
            "see": self.objects,}
        
        # Disable head manager
        head_mgr = NavigationCameraMgr()
        head_mgr.head_mgr_as("disable")
        
        # Movement Parameters
        self.head_lr = 0.0
        self.head_ud = -0.5
        self.head_move_step_size = .15
        self.turn_step_size = 0.2
        self.move_step_size = 0.2
        self.handle_camera_direction(['center'])

    def benchmark_state_callback(self, data):
                     
        if data.benchmark_state == BenchmarkState.STOP:
            rospy.loginfo("STOP")
        elif data.benchmark_state == BenchmarkState.PREPARE:
            rospy.loginfo("PREPARE")
            try:
                time.sleep(5)
                self.prepare()
            except:
                rospy.loginfo("Failed to reply PREPARE")
        elif data.benchmark_state == BenchmarkState.EXECUTE:
            rospy.loginfo("EXECUTE")


    def hearCommand_callback(self,data):
        rospy.loginfo('Heard a command')
        raw = str(data)
        lower = raw.lower()
        text = lower.split('~')[0]
        speech = text.split(':')[1]
        words = speech.split(' ')[1:]
        rospy.loginfo(speech)
        words = [x for x in words if x!='the']
        #for w in range(len(words)):
        #    if words[w] in ["room","chair","cabinet"] :
        #        words[w] = words[w-1]+'_'+words[w]
        #        words.pop(w-1)
        possible_verbs = self.tbm1_commands_dict.keys()
        if words[0] in possible_verbs:
            valid_command = True
            verb = words[0]
            subject = words[1:]
            #TODO parse rooms into single words (e.g. dining room into dining_room)
        else:        #self.pub_move = rospy.Publisher('/hearts/controller/move', Twist, queue_size = 10)        
            valid_command=False
            self.pub_talk.publish("Invalid command please try again")
            return
        if verb == "move" or verb =="turn":
            self.handle_moves(verb,subject)
        elif verb == "go":
            self.handle_destinations(subject)
        elif verb == "look":
            self.handle_camera_direction(subject)
        elif verb == "see":
            self.handle_picture_taking(subject) 
        elif verb == "look":
            self.handle_camera_direction(subject)    
        elif verb == "end":
            self.execute()
        return 

    #SEE the [bedroom, bathroom, hall, entrance] door is [open, closed]
    #SEE the [couch, bed, chair, lamp (furniture)] in the [room]
    #SEE the [coke, biscuits (object)] in the [room] on the [furniture]

    def handle_picture_taking(self,subject):
        thingy = MoveBaseActionFeedback()
        thingy.feedback.base_position.pose.position.x = 2.0
        thingy.feedback.base_position.pose.position.y = 2.0
        thingy.feedback.base_position.pose.orientation.w = 1.0
        self.pub_dummy.publish(thingy)
        rospy.loginfo('Seen something')
        rospy.loginfo(subject)
        #############################################################################
        if 'door' in subject: #Door option: [open, closed] door BETWEEN [room] and [room]
            if 'open' in subject:
                open_status = 'true'
            else:
                open_status = 'false'
            if 'bedroom' in subject:
                thing_id = 'door_bedroom'
                room1 = 'bedroom'
                room2 = 'living_room'
            if 'bathroom' in subject: 
                thing_id = 'door_bathroom'
                room1 = 'bathroom'
                room2 = 'kitchen'
            if 'hall' in subject: 
                thing_id = 'door_hall'
                room1 = 'hall'
                room2 = 'kitchen'
            else: 
                thing_id = 'door_entrance'
                room1 = 'hall'
                room2 = 'outside'
            
            line1 = self.linewriter('type',[thing_id,'door'])
            line2 = self.linewriter('connects',[thing_id,room1,room2])
            line3 = self.linewriter('isOpen',[thing_id,open_status])
            to_write = [line1,line2,line3]
        #############################################################################
        elif subject[0] in self.objects: #[object] in [room] (room) on [furniture] (furniture)
            if len(subject)<5:
                self.pub_talk.publish("Invalid object please try again")
            thing_id = subject[0] #TODO add matching to IDs
            if thing_id == 'gum':
                thing_id = 'chewing gum'
            position_string = self.object_position() #TODO get actual position
            self.pub_pic.publish(subject[0]+'.jpg')
            item = thing_id
            
            if subject[3] == 'room':
                room = subject[2]+'_'+subject[3]
                subject.pop(3)
            else:
                room = subject[2]
            if len(subject)==6:
                furniture = subject[4]+'_'+subject[5]
            elif len(subject)==7:
                furniture = subject[4]+'_'+subject[5]+'_'+subject[6]
            else:
                furniture = subject[4]
            line1 = self.linewriter('type',[thing_id, item])
            line2 = self.linewriter('in',[thing_id, room])
            line3 = self.linewriter('on',[thing_id, furniture])
            line4 = self.linewriter('position ',[thing_id, position_string])
            line6 = self.linewriter('picture',[thing_id,thing_id+'.jpg'])
            to_write = [line1,line2,line3,line4,line6]
        ###################################################################################
        elif subject[0] in self.furniture: # [couch, bed, chair, lamp (furniture)] in [room]
            if subject[1]=='in':
                thing_id = subject[0]
            else:
                thing_id = subject[0]+'_'+subject[1]
                subject.pop(1)
            if len(subject)==4:
                room = subject[2]+'_'+subject[3]
            else:
                room = subject[2]
            if thing_id == 'armchair':
                thing_id = 'arm_chair'
            if thing_id == 'night' or thing_id == 'night_stand':
                thing_id = 'nightstand'
            line1 = self.linewriter('type',[thing_id,thing_id]) 
            line2 = self.linewriter('in',[thing_id,room])
            to_write = [line1,line2]
        else:
            self.pub_talk.publish("Invalid noun please try again")
            return
        f = open(self.outfolder+'sementic_map.txt','a+')
        for line in to_write:
            f.write(line+'\n')
            rospy.loginfo(line)
        f.close()


    def linewriter(self, descriptor, options_list):
        text = descriptor + '('
        for option in options_list:
            text = text+option+', '
        text = text[:-2]+').'
        return(text)




    def handle_camera_direction(self,subject):
        command = JointTrajectory()
        command.joint_names = ["head_2_joint","head_1_joint"]
        point1 = JointTrajectoryPoint()
        #point2 = JointTrajectoryPoint()
        
        point1.velocities = [0.0,0.0]
        #point2.velocities = [0]
        
        point1.time_from_start = Duration(2.0,0.0)
        rospy.loginfo(subject)
        if subject[0] == 'up':
            self.head_ud = self.head_ud + self.head_move_step_size
        if subject[0] == 'down':
            self.head_ud = self.head_ud - self.head_move_step_size
        if subject[0] == 'left':
            self.head_lr = self.head_lr + self.head_move_step_size
        if subject[0] == 'right':
            self.head_lr = self.head_lr - self.head_move_step_size
                    
        rospy.loginfo(self.head_ud)
        point1.positions = [self.head_ud, self.head_lr]
        command.points = [point1]
        self.pub_head.publish(command)
        return

    def handle_destinations(self,subject):
        rospy.loginfo('Heard a destination')
        destination_list = [x for x in subject if x!="to"]
        destination = ' '.join(destination_list)
        self.pubGoal.publish(destination)
        return

    def handle_moves(self,verb,subject):
        rospy.loginfo('Moving')
        rospy.loginfo(verb)
        rospy.loginfo(subject[0])
        #rospy.loginfo(subject[1])
        if len(subject)>1:
            if subject[1] == "more":
                distance = 5
            elif subject[1] == "less":
                distance = 1
            else:
                distance = 1
        else:
            distance = 2
        rospy.loginfo(distance)
        d=0
        t0 = rospy.get_time()
        while(d<distance):
            if(verb == 'move' and subject[0] == 'forward'):
                self.send_directions(self.move_step_size,0)
            elif(verb=='move' and subject[0] == 'backward'):
                self.send_directions(-1*self.move_step_size,0)
            elif(verb == 'turn' and subject[0] == 'left'):
                self.send_directions(0,self.turn_step_size)
            elif(verb == 'turn' and subject[0] == 'right'):
                self.send_directions(0,-1*self.turn_step_size)
            elif(verb == 'turn' and subject[0] == 'around'):
                self.send_directions(0,-1*self.turn_step_size)
                distance = 20
            d = rospy.get_time()-t0
          
    def send_directions(self,straight,turn):
        rospy.loginfo('Sending out a direction')
        t = Twist()
        ### insert intended values for movements here to fed to navigation here ###
        t.linear.x = straight
        t.linear.y = 0
        t.linear.z = 0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = turn

        self.pub_move.publish(t)
        rospy.sleep(0.01)
    
    def object_position(self):
        ####Read in base position
        d = 1 #distance from object
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        t = self.current_pose.orientation.w
        x2 = round(x + d*cos(t),2)
        y2 = round(y + d*sin(t),2)
        z2 = .5
        l = '['+str(x2)+','+str(y2)+','+str(z2)+']'
        ####Calculate approximate object position
        #object_position = currentpose + position
        return(l)


    def current_pose_callback(self, data):
        self.current_pose = data.feedback.base_position.pose

        return




if __name__ == '__main__':
    rospy.init_node('know_my_home', anonymous=True)
    rospy.loginfo("know my home controller has started")
    controller = Controller()
    rospy.spin()
