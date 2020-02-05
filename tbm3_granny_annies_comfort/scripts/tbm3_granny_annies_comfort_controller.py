#!/usr/bin/env python
##############################################################################################
# Author  : Derek Ripper/Joe Daly
# Created : Dec 2017
# Purpose : To fulfil ERL-SR competition Bench Mark  - TBM3 Grannie Annies comfort
#           Written for the 22-26 Jan 2018 ERL competition in Edinburgh.
#
##############################################################################################
# Updates :
#
# Oct 2018 in Madrid:
#         - generic controller introduced
#         - new language processor introduced for the June 2018 rule book
# Jan/Feb 2019 - BRL competition
#         - more debugging for logic errors
# Oct 2019 - Revist TBM3 correct logic , add GA postion to data from Tablet
#
# Jan 2020 - Derek
#           Remove the frigs that allowed us to stop using subscribers for listening for
#           Commands and Anwsers. These were put in to try and bug fix at BRL for the FEB 2019
#           ERL competition. With hindsight we went down the wring track! and blamed the 
#           introduction of the generic controller for listening issues.
#
#           ALSO - have remove all code relating to the Rulebook for TBM3 as used in the last
#           competition in Edinburgh Jan 2018. Switching on lights,operating blinds etc,etc,
#           was dropped from the Rulebook for Madrid Oct 2018 onwards.
#         - 
#############################################################################################
import rospy
import time
import std_srvs.srv
from   std_msgs.msg           import Empty, String, Bool
from   geometry_msgs.msg      import Pose2D, Pose, Twist, PoseStamped
from   roah_rsbb_comm_ros.msg import BenchmarkState
from   roah_rsbb_comm_ros.srv import Percentage
from random import *
from python_support_library.generic_controller import GenericController

import python_support_library.text_colours as TC
import language_v2 as nlp

prt = TC.tc() # generally used instead of print stmt to colorise messages appropriately

class ControllerTBM3(GenericController):

    def __init__(self):
        # init the generic stuff from GenericController
        super(ControllerTBM3, self).__init__()

        #Publishers
        self.pub_twist =   rospy.Publisher('/mobile_base_controller/cmd_vel', Twist,  queue_size=10)
        self.pose_2d_pub = rospy.Publisher('hearts/navigation/goal',          Pose2D, queue_size=10)

        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)

        #Services
        self.prepare =              rospy.ServiceProxy('/roah_rsbb/end_prepare',          std_srvs.srv.Empty)
        self.execute =              rospy.ServiceProxy('/roah_rsbb/end_execute',          std_srvs.srv.Empty)
        self.user_location_service =rospy.ServiceProxy('/roah_rsbb/tablet/map',           std_srvs.srv.Empty)

        #ROS parameters
        self.IROBOT = rospy.get_param("robot_in_use")

        #Initialise variables
        self.user_location = None
        self.FirstCall     = True  #used to store GA's position on once only when first told via tablet

        # speech processing and load up competition data
        self.analysis = nlp.Analysis(self)
        self.analysis.getcompdata()

        # Granny Annies position in judges coord system
        # h for high value and l for low value ie the range
        self.jlocxh = [ 0.00,  1.50]
        self.jlocxl = [-4.00,  0.00]
        self.jlocyh = [-0.50, -4.20]
        self.jlocyl = [-7.20, -5.50]

    def listen4cmd(self,status):

        if status == 'on' :
            prt.info('***** Listening for a COMMAND')
            #self.toggle_stt('on')
            self.sub_cmd=rospy.Subscriber("/hearts/stt", String, self.heardCommand_callback)
            #self.toggle_stt('off')

        else:
            #self.toggle_stt('off')
            prt.info('***** NOT! Listening for a COMMAND')
            self.sub_cmd.unregister()

        return

    def listen4ans(self,status):
        if status == 'on' :
            prt.info('***** Listening for an ANSWER revised code')
            self.toggle_stt('on') #this makes mic work ...why???

            prt.debug("##### In listen4ans: before subscriber")
            self.sub_ans=rospy.Subscriber("/hearts/stt", String, self.heardAnswer_callback)
            prt.debug("##### In listen4ans: after  subscriber")
            #self.toggle_stt("off")

        else:
            #self.toggle_stt('off')
            self.sub_ans.unregister()
            prt.info('***** NOT! Listening for an ANSWER')

        return

    def heardAnswer_callback(self,data):
        speech = str(data)
        speech = speech.lower()
        speech = speech.replace('"','')
        rospy.loginfo('*** Heard an answer : '+speech+'\n')

        words  = speech.split(' ')
        prt.debug("words list in YES/NO section")
        for item in words:
            prt.debug("yes/no words list:>"+item+"<")
        if 'yes' in words:
            self.say("OK then I will do that now")

            self.analysis.executeobjectives(self.theobjectives)

            self.say("The tasks are now completed to your satisfaction we trust")
            prt.todo(" **************Stop Program nicely  here!*****************")
            prt.warning("TBM3 controller: now quitting python!")
            quit()
            # Turn off subscribing to any /hearts/stt topics
            self.listen4ans('off')
            self.listen4cmd('off')

        elif 'no' in words:
            self.say("OK I will forget your last command. Please give me another command")

            # re-establish subscribers
            self.listen4ans('off')
            self.listen4cmd('on')

        else:
            self.say("Please answer with yes please or no thank you")
            self.listen4ans('on')
        return

    def heardCommand_callback(self,data):
        self.listen4cmd('off')
        prt.debug("in heardcommand_callback - speech: "+str(data))
        speech = str(data)
        speech = speech.lower()
        rospy.loginfo('*** Heard a command\n'+speech+'\n')

        # remove the ROS msg "Data:" value from speech
        item = 'data:'
        if item in speech:
            speech = speech.replace(item,'')

        # check that text has been returned
        if "bad_recognition" in speech:
            self.say("Sorry, no words were recognised. Please repeat.")
            self.listen4cmd('on')
            return

        ###### NEW CODE for speech processing    ######

        # use GA's location from her tablet to update the "founditems" list
        x =      self.user_location.x
        y =      self.user_location.y
        theta =  self.user_location.theta
        self.analysis.InitialUserLoc("user",[x,y,theta])

        prt.info("###START: derive the 3 objectives from the spoken text")
        commandcount, self.theobjectives = self.analysis.defineobjectives(speech)
        prt.info("###END  : derive the 3 objectives from the spoken text")

        if commandcount == 3 :
            # prt.debug("***********printme    all fields**************************************")
            # self.theobjectives[0].printme()
            # self.theobjectives[1].printme()
            # self.theobjectives[2].printme()
            # prt.debug("***********printme    all fields***************************************")
            prt.debug("***********printme_final    *******************************************")
            self.theobjectives[0].printme_final()
            self.theobjectives[1].printme_final()
            self.theobjectives[2].printme_final()
            prt.debug("***********printme_final END*******************************************")

            talkback      = self.analysis.getconfirmationtext(self.theobjectives)

            prt.info("command count rtn to GA controller = "+str(commandcount))
            prt.info("***** Tiago's confirmation to Granny Annie is: \n"+talkback+"\n")
            self.say("You requested that I "+talkback)

            prt.info("Asking for YES or NO now!")
            self.say("Shall I do this now?")
            self.listen4ans('on')

            #dar self.listen4cmd('off')


        else:
            prt.info("command count = "+str(commandcount)+" so cannot proceed")
            txtcmds = self.num2text(commandcount)
            self.say("I have received "+txtcmds+" but expected three. Please repeat command.")
            self.listen4cmd('on')

        return

    def num2text(self,number):
        if      number == 0:
            txt = "zero commands"
        elif number == 1:
            txt = "one command"
        elif number == 2:
            txt = "two commands"
        elif number  > 3:
            txt = "more than three commands"

        return txt

    ### When receiving a message from the "roah_rsbb/benchmark/state" topic, 
    ### will then publish the corresponding state to "roah_rsbb/messages_save"
    def benchmark_state_callback(self, data):
        if data.benchmark_state == BenchmarkState.STOP:
            rospy.loginfo("STOP")
        elif data.benchmark_state == BenchmarkState.PREPARE:
            rospy.loginfo("PREPARE")
            try:
                rospy.sleep(0.1)
                self.prepare() # END of PREPARE msg to service
            except:
                rospy.loginfo("Failed to reply PREPARE")

        elif data.benchmark_state == BenchmarkState.EXECUTE:
            rospy.loginfo("EXECUTE")
            self.main()

    def wait_for_call(self):
        rospy.loginfo("***** Waiting for call from GA")
        self.wait = False
        sub = rospy.Subscriber("/roah_rsbb/tablet/call", Empty, self.tablet_callback)
        while self.wait == False:
            rospy.sleep(0.1)
        print("***** Call recd from GA")
        sub.unregister()

    def wait_for_user_location(self):
        rospy.loginfo("Waiting for user location")
        self.user_location = None
        sub = rospy.Subscriber("/roah_rsbb/tablet/position", Pose2D, self.user_location_callback)
        rospy.wait_for_service('/roah_rsbb/tablet/map')

        self.user_location_service()
        rospy.loginfo("going to while loop")
        while self.user_location is None:
            rospy.sleep(0.1)
            rospy.loginfo("Waiting for user location in while loop")
        sub.unregister()


        # Callback functions
    def tablet_callback(self, msg):
        self.say("granny annie has called for attention")
        self.wait = True

    def user_location_callback(self, msg):
        # this is Granny Annie's location being the 'user'!

        rospy.loginfo("Waiting for user location callback")
        rospy.loginfo(msg)

        self.user_location = msg
        #####dself.analysis.Objective.storefoundloc("user", msg )
        '''
        print("msg.x     from GA tablet: "+str(msg.x))
        print("msg.x     from GA tablet: "+str(msg.y))
        print("msg.theta from GA tablet: "+str(msg.theta))



        #Edinburgh 2018 code for remapping test bed coords to the Tiago ones
        for idx in range (0,2):
            if  msg.x > self.jlocxl[idx] and msg.x < self.jlocxh[idx] and \
                msg.y > self.jlocyl[idx] and msg.y < self.jlocyh[idx]     :

            # assign the coords in our system
            # msg.x = self.ulocx[idx]
            # msg.y = self.ulocy[idx]
            # msg.theta = self.uloct[idx]

            found = True

        if not found :
            print("Re-mapping for Grany Annie location failed!")
            print("\n***** STOPPING PROGRAM *****\n")
            quit()
        '''
        prt.result(" Granny Annies location:")
        prt.result(" X     : "+str(self.user_location.x))
        prt.result(" Y     : "+str(self.user_location.y))
        prt.result(" theta : "+str(self.user_location.theta))

        return

    ## Navigation Functions
    def move_to_pose2D(self, target_location_2D):
        ##publish granny annie's location
        rospy.loginfo("Moving to Pose2D")

        self.pose_2d_pub.publish(target_location_2D)

        self.wait_to_arrive(5)

    def device_operationsself(self):
        pass

    def move_robot_to_coords(self,key,coords,trys):
        # indirection code to allow deveopment with no robot attached

        if self.IROBOT:
            prt.warning("ROBOT moving to location of: "+key)
            prt.warning("Coords are: "+str(coords))
            self.move_to_coords(coords,trys)
        else:
            prt.warning("NO ROBOT available for software to control!")
            prt.warning("ROBOT will NOT moving to Location of: "+key)
            prt.warning("Coords are: "+str(coords))
        return

    def move_robot_to_location(self,location,trys):
        # indirection code to allow deveopment with no robot attached

        if self.IROBOT:
            prt.warning("ROBOT moving to location : "+location)
            self.move_to_location(location,trys)
        else:
            prt.warning("NO ROBOT available for software to control!")
            prt.warning("ROBOT will NOT moving to : "+location+" with "+str(trys) )

    def move_to_coords(self, coords, num_retries):
        #copied from move_to_location #Derek
        '''
        Publish coords to move to to /hearts/navigation/goal/location and wait
        for update on /hearts/navigation/status (either "Success" or "Failure")

        If move not succesful, retry given number of times, changing orientation
        by 1 radian on each retry.
        '''
        #self.cost_clear()
        rospy.loginfo("moving to coords \"" + str(coords) + "\" (" + str(num_retries) + ")")
        msg = Pose2D()
        prt.debug("######## COORDS HERE ########")
        print coords
        print type(coords)

        if isinstance(coords, list):
            msg.x     = coords[0]
            msg.y     = coords[1]
            msg.theta = coords[2]
        else :#TODO this is dangerous, might not be right message type
            msg.x     = coords.x
            msg.y     = coords.y
            msg.theta = coords.theta
        #########
        ###self.pub_location_goal.publish(msg)### as used in mve to location
        #########
        self.pose_2d_pub.publish(msg)

        arrive_success = self.wait_to_arrive(num_retries)

        if arrive_success == False:
            rospy.loginfo("ERROR movement has failed")
            self.say("Sorry, I am unable to move to get there")
            #TODO should an actual error be thrown here?
            print coords
            return False
        else:
            self.say("I have arrived")
            return True


    def main(self):
        print ("\n***** MAIN Executing *****\n")
        prt.debug(" I-Robot :"+str(self.IROBOT))
        #go to home position
        self.move_robot_to_location("home",1) #TODO check number of retries?

        #wait for call
        self.say("Waiting to be called by granny annie.")
        self.wait_for_call()
        prt.debug("------  IROBOT : "+str(self.IROBOT))
        #request location
        #self.say("Waiting for granny annie's location") - removed as delay seems to prevent user location callback from firing
        self.wait_for_user_location()

        #navigate to the user's location
        self.say("I am on my way to you.")
        prt.todo("Remove comments for navigation to GA")
        trys=5
        self.move_robot_to_coords('User',self.user_location,trys)
        prt.todo("retries fr GA arriving???")

        self.say("How can I help you today? Please give me a command")

        self.listen4cmd('on')
        prt.error("dropped out of def main")
        rospy.loginfo("End of MAIN programme")


if __name__ == '__main__':


    rospy.init_node('annies_comfort', anonymous=False)
    rospy.loginfo("annies comfort controller has started")
    controller = ControllerTBM3()
    rospy.spin()
