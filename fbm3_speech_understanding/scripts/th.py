#!/usr/bin/python
# Filename: th.py
# Created: 07 May 2017
# Author : Derek Ripper
# Purpose: To read the ground_truth.csv file and run speech2text (stt.py) in
#          test harness mode for all contained wav files.
################################################################################
# Updates:
# 13 Nov 2017 Derek - updated changed TOPIC name from Text_4_CFR to hearts/stt
#                   - variable added for "ground truth" delimeter.
#                     This enables us to use the ERL so called  goldStandard files
#                     to drive the "TH" mode.
#                   - Removed small amounts of obsolete code and some message to screen
#                     improvements    
#                     
# 24 Oct 2017 Derek - changed  delimiter in ground truth file to the unix pipe "|"
#                     This circumvented a problem with losing the first quote mark.
#                     Hence we can now use teh ERL groundtruth file (goldStandard in 
#                     Lisbon 2017 competition) without modification. Also must call
#                    it a text file ).txt   
# 22 Aug 2017 Derek- remove rospy.sleep(5) stmt in loop_wavfiles routine. Originally
#             this was needed before I installed the tagging system for topics. Now they 
#             cannot get out of sync! 
#             Also removed topic counters as they can no longer be seen as
#             all wav files are published first due to 5 sec time delay being removed. 
#
# 21 Aug 2017 Derek - added code to only read .wav files from ERLUSB mem stick
#
# 17 Aug 2017 - Derek change pointer for storing CFR results to achieve excel formulas
#               in TH results file. 
# 11 Aug 2017 - Derek
# Correct callback1 as topic Text_4_CFR now has 2 data elements being:
# Text from speech ~ Audio file name needed for ERL competition results
#
# 20 june 2017 - derek
# after running under ROS proven - found published topics could arrive in
# in a random order.
# To circumvent this all TOPICS are prefixed with an integer index (#xxxx#)
# that directly relates the publish topic to the subscribed one. 
# See tag_topics.py class definition that handles adding/removing tags and
# storing the final answers.
#
# 30 May 2017 - Derek
# take prototype routine and adjust to use in ROS env.
################################################################################

import csv
import os
import tag_topics  as TT
import rospy
from   std_msgs.msg import String


o_tt=TT.tag_topics()

class th:
    def __init__(self):    
        rospy.init_node("TH_Driver",anonymous=True)
        self.run_mode= rospy.get_param("SR_TH")
        self.delimiter =  rospy.get_param("SR_GROUNDTRUTHDELIM")
       
        self.txt_cnt = 0
        self.cfr_cnt = 0

        # set up a publiser
        self.pub = rospy.Publisher("Wav_FileIn",String,queue_size=10)

    def listeners(self):
        # set up subscriber for returned TOPIC  
        self.sub1 = rospy.Subscriber("hearts/stt", String, self.callback1)
        
        # set up subscriber for returned TOPIC  
        self.sub2 = rospy.Subscriber("CFR_Out",    String, self.callback2)
    
    def callback1(self,data): # process hearts/stt topic
        self.txt_cnt += 1
        phrase, wavname = data.data.split('~')
        o_tt.store_result(2, phrase.strip())
        rospy.loginfo(rospy.get_name()+": Frm  hearts/stt data.data is: "+data.data) 

    def callback2(self,data): # process CFR_out topic
        self.cfr_cnt += 1
        o_tt.store_result(5, data.data)
        rospy.loginfo(rospy.get_name()+": Frm T2CFR data.data is: "+data.data)
  
    def get_run_mode(self):
        return self.run_mode

    def bld_csv_file(self, path):
        tempcsvfile = self.DATAPATHOUT+"ERL_audio_input_data.csv"
        fh = open(tempcsvfile,'wb')
        csvwrite = csv.writer(fh,delimiter='|',quotechar='"')
        for entry in os.listdir(path):
            if entry.endswith(".wav"):
               row = [path+entry,"NO_TRAN","NO_CFR"]
               csvwrite.writerow(row)     
        return  tempcsvfile


# Speech to Text engine -- select from google, ibm or sphinx
# This is now set within the ROS launch file
    def loop_wavfiles(self): 
        self.DATAPATHOUT = rospy.get_param('SR_BRL_DATAPATHOUT')
        GROUNDTRUTH = rospy.get_param('SR_GROUNDTRUTH')
        ERLAUDIOPATH= rospy.get_param('SR_ERL_AUDIO_DIR')
        BRLAUDIOPATH= rospy.get_param('SR_BRL_AUDIO_DIR')
        filecnt = 0  
        #check if ERL-SR competition audio path has been set
        if ERLAUDIOPATH == '':
            filein = GROUNDTRUTH
            print("ground truth file: "+ filein)
            csvdelimter=self.delimiter
        else:
            # scan ERL audio files and build temp csv file in TH format
            # same format as ground_truth.cav file
            filein = o_th.bld_csv_file(ERLAUDIOPATH)
           
            # negate AUDIOPATH as used in BRL mode only
            BRLAUDIOPATH=''
            csvdelimter="|"

        with open(filein, mode='r') as csvinputfile:

            csvreader = csv.reader(csvinputfile, delimiter=csvdelimter)
            len_csv = sum(1 for row in csvreader)
            rospy.loginfo(rospy.get_name()+": No of rows in GROUNDTRUTH file: "\
                          +str(len_csv))

            # return to top of file 
            csvinputfile.seek(0)
          
            # initialise  array to store all results
            o_tt.bld_arr(len_csv)
    
            for row in csvreader:
                filecnt += 1
                wavfile  = BRLAUDIOPATH + row[0].replace('\\','/')
                trueTXT  =                row[1].strip()
                trueCFR  =                row[2].strip()
                o_tt.store_inputs(filecnt,'=HYPERLINK("'+wavfile+'")',
                                           trueTXT,
                                           trueCFR)

                # pass wavfile to "speech to text" routine	
                rospy.loginfo(rospy.get_name()+": *** Wav file seq # *** = " \
                              +str(filecnt)) 
                rospy.loginfo(rospy.get_name()+": Pub wavfile= "+wavfile)

                #prefix wavfile with sequential file number ie row number
                wavfile = o_tt.add_key(filecnt,wavfile)
                self.pub.publish(wavfile)
                
                rospy.loginfo(rospy.get_name()+": True Text is: " + trueTXT+ \
                              " True CFR is: "+trueCFR+"\n")
       
               
        rospy.loginfo(rospy.get_name()+" : Total audio files found = " \
                      +str(filecnt))
        # Don't exit python code until the callbacks have been processed for
        # all the returned Topic data
        toggle = True 
        while toggle:
           if self.txt_cnt == len_csv and self.cfr_cnt == len_csv : 
               print("\n***** "+str(len_csv)+" audio files have been processed in Test harness mode")
               toggle = False  
                           
        #write ALL collected results to file
        o_tt.write_results_to_file(self.DATAPATHOUT+'TH_Speech_recognition_results.txt')
   
        print("*****    Use CTRL-C to kill ROS Node \n")

##################################
##### end of class "th" definition
##################################

o_th = th()  

def main(): 
    o_listen= o_th.listeners()
    rospy.sleep(5)
    o_th.loop_wavfiles()

if __name__ == '__main__':
    main()	

rospy.spin()	
				
