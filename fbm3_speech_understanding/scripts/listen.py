#!/usr/bin/python
import rospy
from   std_msgs.msg import String
from sound_play.msg import SoundRequest

class th:
    def __init__(self):    
        pass  
        #self.run_mode= rospy.get_param("SR_TH")
        #rospy.loginfo(rospy.get_name()+": Run Mode is: "+self.run_mode)

    def listeners(self):
        # set up subscriber for returned TOPIC  
        self.sub0 = rospy.Subscriber("Wav_FileIn", String, self.callback0)


        # set up subscriber for returned TOPIC  
        self.sub1 = rospy.Subscriber("Text_4_CFR", String, self.callback1)
    
        # set up subscriber for returned TOPIC  
        self.sub2 = rospy.Subscriber("CFR_Out",    String, self.callback2)

        # set up subscriber for returned TOPIC  
        self.sub3 = rospy.Subscriber("robotsound", SoundRequest, self.callback3)

        # set up subscriber for returned TOPIC  
        self.sub4 = rospy.Subscriber("/hearts/stt", String, self.callback4)

        rospy.spin()

    def callback0(self,data):
        #
        self.out0=data.data
        rospy.loginfo(rospy.get_name()+": Wav_Filin     data.data is: "+self.out0)

    
    def callback1(self,data):
        #
        self.out1=data.data
        rospy.loginfo(rospy.get_name()+": Text_4_CFR -  data.data is: "+self.out1) 
  
    def callback2(self,data):
        #
        self.out2=data.data
        rospy.loginfo(rospy.get_name()+": CFR_OUT    -  data.data is: "+self.out2)

    def callback3(self,data):
        #
        self.out3=data.arg
        print(type(data.arg))
        rospy.loginfo(rospy.get_name()+": robotsound    -  data.SAY is: "+self.out3)

    
    def callback4(self,data):
        #
        print("Callback 4 ......")
        self.out4=data.data
        rospy.loginfo(rospy.get_name()+": hearts/tts -     data.data is: "+self.out4)
 
if __name__ == '__main__':
    print("\nRunning Listeners .....\n")
    rospy.init_node("TEST_LISTEN",anonymous=True)   
    o_th=th()
    o_th.listeners()
    print("\nRunning Listeners .....\n")

    while not rospy.is_shutdown():

        try:
            pass
        except KeyboardInterupt:
            print("Shutting down node : TEST_LISTEN node")


				
