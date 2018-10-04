#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import sys


def talker(speech):
  pub = rospy.Publisher('/hearts/stt', String, queue_size=10)
  rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(10) # 10hz

  #while not rospy.is_shutdown():

  rospy.loginfo(speech)
  pub.publish(speech)
  rate.sleep()
   
arg = sys.argv[1]
print("arg: "+arg)
if   arg == "task1":
  speech = "Locate Tracy, lead her to the bedroom, and bring me an apple from the kitchen cabinet."
elif arg == "task2":
  speech = "Locate the bottle, place it in the bucket, and take John from the bedroom to the exit."
elif arg == "task3":
  speech = "Take  me to the bedroom, find my glasses, and bring me a pear from the kitchen table."
elif arg == "task4":
  speech = "Give me the cup on the coffee table, find John, and follow him."
elif arg == "task5":
  speech = "Hi Tiago, please will you go and find John, I need you to take him to the exit and then get me a bottle of water from the kitchen cabinet, thanks!"
elif arg == "task6":
  speech = "Search for the tea pot, Get my glasses, and Bring me a pear from the coffee table."
elif arg == "task7":
  speech = "Find for john , Get my glasses, and Bring me a pear from the coffee table."
elif arg == "task8":
  speech = "find the pear, get me the pear and find for my glasses"
elif arg == "task9":
  speech = "close blinds"
elif arg == "yes":
  speech = "yes please"
elif arg == "thk":
  speech = "No thank you"

try:
    talker(speech)
except rospy.ROSInterruptException:
    pass