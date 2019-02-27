import numpy as np
import string
import re
import json
import rospy
from std_msgs.msg      import String

import python_support_library.text_colours as TC

prt = TC.tc()

# class tc: # Termianl "text colour" control
#     HEADER    = '\033[95m' # purple
#     OKBLUE    = '\033[94m' # blue
#     OKGREEN   = '\033[92m' # green
#     WARNING   = '\033[93m' # yellow
#     FAIL      = '\033[91m' # red
#     ENDC      = '\033[0m'  # revert to normal text
#     BOLD      = '\033[1m'
#     UNDERLINE = '\033[4m'


class Objective:
    instances  = 0
    founditems = {} # coords from Tiago of object and/or person found
                    # data format  {name : [x,y,theta]}

    def __init__(self,sentence,com,brlcom,comtype,analysis, tbm3ctrler):
        self.tbm3ctrler = tbm3ctrler
        #publishers
        self.tts_pub = rospy.Publisher("/hearts/tts", String, queue_size=10)

        Objective.instances += 1

        self.analysis   = analysis

        self.instance   = Objective.instances

        self.sentence   = sentence
        self.command    = []
        self.brlcommand = []
        self.comtype    = []
        self.person     = []
        self.object     = []
        self.location   = []
        self.fromLocation = []
        self.toLocation   = []
        #self.furniture = []
        self.reference        = []
        self.locationModifier = []
        self.command.append(com)
        self.brlcommand.append(brlcom)
        self.comtype.append(comtype)
        self.confirmationtext = ""
        self.success = False

        self.load_json_coords()

        #from a ROS parameter in caller
        self.IROBOT = self.tbm3ctrler.IROBOT


    # def say(self, text):
    #     ''' Publish text to tts_pub where text is then spoken aloud by tiago'''
    #     rospy.loginfo("saying \"" + text + "\"")
    #     rospy.sleep(1)
    #     self.tts_pub.publish(text)
    #     rospy.sleep(5)


    
    def load_json_coords(self): 

        jsonfilein = rospy.get_param("locations_json")
        prt.debug("###### locations.json: "+jsonfilein)
        with open(jsonfilein) as fh:
            data = json.load(fh)

        # get top level keys - ie the location names
        keylist = data.keys()
        keylist.sort()

        for key in keylist: 
            x          = data[key]['x']
            y          = data[key]['y']
            theta      = data[key]['theta']
            coordslist = [x,y,theta]

            self.storefoundloc(key,coordslist)

        prt.todo("GET proper coords of GA's location fom her tablet?????")
        coordobjectiveslist=[111,222,-99.9]
        self.storefoundloc('user',coordslist)    

    def parse(self):
        #reads sentence and stores key words

        #check for 2 (or more) word locations & objects
        self.sentence = self.analysis.process_locations(self.sentence)
        self.sentence = self.analysis.process_objects  (self.sentence)
        splitList = self.sentence.split()

        for word in splitList:
            test = word 
            
            if test in self.analysis.locations:
                self.location.append(word)
            #elif test in furnitures:
            #    self.furniture.append(word)
           
            elif test in self.analysis.objects:
                self.object.append(word)
            elif test in self.analysis.people:
                self.person.append(word)
            elif test in self.analysis.references:
                self.reference.append(word)
            elif test in self.analysis.locModifiers:
                self.locationModifier.append(word)

        #uses modifier words 'to' and 'from' to fill in 'to' and 'from' locations - important in "take thing from x to y" tasks
        count = 0
        while len(self.locationModifier) > 0:
            word = self.locationModifier[0]
            #print "word: " + word
            if word == "from":
                try:
                    self.fromLocation.append(self.location[count])
                    del self.locationModifier[0]
                except IndexError:
                    #print "No location matching 'from' modifier"
                    del self.locationModifier[0]
            elif word == "to" or word == "in":
                try:
                    self.toLocation.append(self.location[count])
                    del self.locationModifier[0]
                except IndexError:
                    #print "No location matching 'to' modifier"
                    del self.locationModifier[0]
            count = count + 1

    def process_objective(self):
    
        #ensure underscore removed from location type lists before data passed on to Tiago
        if len(self.location) > 0:
             self.location[0]  = self.location[0].replace('_',' ')
        if len(self.fromLocation) > 0:
             self.fromLocation[0]  = self.fromLocation[0].replace('_',' ')
        if len(self.toLocation) > 0:
             self.toLocation[0]  = self.toLocation[0].replace('_',' ')

        # define the "FROM" location if not assigned thru a locModifier     
        if len(self.toLocation) == 0 :
            self.fromLocation = self.location

        #  below self.command[0] & comtype[0] will be changed to the
        #  BRL equivalent before comparison takes place
        brl_com, com_type = self.get_brl_com(self.command[0])
        self.command[0] =  brl_com
        self.comtype[0] =  com_type

        ## SEARCH for OBJECT ## - when there is no "fromLocation"  for 'find/get an object' then use the ones
        #                         given in the ERL data (ie the 3 possible locations)
        if (brl_com == 'find' or brl_com == 'get') and len(self.object) > 0 and len(self.location) == 0:
            loc0,loc1,loc2 = self.analysis.get_obj_per_loc(self.object[0])
            self.fromLocation.append(loc0)
            self.fromLocation.append(loc1)            
            self.fromLocation.append(loc2)


        ## SEARCH for PERSON ## - when there is no "fromLocation"  for 'find/get a  person'  then use the ones 
        #                         given in the ERL data (ie the 3 possible locations)
        if (brl_com == 'find' or brl_com == 'get') and len(self.person) > 0 and len(self.location) == 0:

            loc0,loc1,loc2 = self.analysis.get_obj_per_loc(self.person[0])
            self.fromLocation.append(loc0)   
            self.fromLocation.append(loc1)
            self.fromLocation.append(loc2)

        ## MANIPULATE and ACCOMMPANY ## - when there is no fromLocation defined it probably comes from  the
        #  previous task. 
        #todo     NB the next 2 IF's related to efore I caged the system t loo up all coords in foundtems
        #          so now shoud just throw up errors as the TO & FROM being  tested should be defined.                         
        if (brl_com == 'get' or brl_com == 'guide' or brl_com == 'follow') and  len(self.fromLocation) == 0:
            self.fromLocation.append('previous task') 


        # if TO location empty assign to person (seems reasonable guess?)
        if len(self.toLocation) == 0 and self.comtype[0] != 's':
            self.toLocation = self.person

        if brl_com == 'follow':
            self.toLocation = ''


        ###################################################################
        
        self.confirmationtext =  self.buildconfirmtext(self.comtype[0])
        return


    def buildconfirmtext(self,action_type):

        wordstring = ""
        if   action_type == 'a':
            wordstring = self.command[0]
            per = 'unknown'
            if self.person:
                per = self.person[0]
                if  per == 'user':
                    per =  'you'  
            wordstring = wordstring +' '+per
            if self.toLocation :
                wordstring = wordstring +' to the '+self.toLocation[0]

        elif action_type == 's':
            wordstring = self.command[0]
            #search target is either an object or person
            if  self.person :
                    per = self.person[0]
                    if  per == 'user':
                        per  = 'your'  
                    wordstring = wordstring +' '+per

            if  self.object:
                    obj = self.object[0]
                    wordstring = wordstring +' '+obj     

            if self.fromLocation :
                if len(self.fromLocation) == 1 and not  'previous' in self.fromLocation[0]:
                    wordstring = wordstring + ' ' + self.fromLocation[0]

        elif action_type ==  'm':   
            wordstring = self.command[0] 
            ##DAR
            if  self.object:
                wordstring = wordstring +' '+self.object[0]    
            if self.fromLocation:
                if len(self.fromLocation) == 1 and not  'previous' in self.fromLocation[0]:
                    wordstring = wordstring +' from the ' +self.fromLocation[0]

            if self.toLocation :
                toLoc      = self.toLocation[0]
                if  toLoc == 'user':
                    toLoc = 'you'  
                wordstring = wordstring +' to '   + toLoc                    

        return wordstring

    def get_brl_com(self, erl_com):    
        # find the equivalent BRL command to the ERL one
        for cmd in self.analysis.commands:
            if erl_com == cmd[1]:
                brl_com = cmd[2]
                comtype = cmd[0]
                break
        #************************
        # **** Special case *****
        #************************
        # "take " can either be a "MANIPULATING" or "ACCOMPANYING" command.
        # Make decision based on context: ie Type of object/person
        if erl_com == 'take':

            if len(self.person) > 0 :
                brl_com = 'guide'
                comtype = 'a'

            if len(self.object) > 0 :             
                brl_com = 'get' 
                comtype = 'm' 

        return brl_com, comtype        

    def printme(self):
        #prints all fields in the objective that have values in them

        print "\n***** Split ACTION *****: "
        print self.sentence
        print "***** command: "
        print self.command
        if len(self.person) > 0:
            print "***** people: "
            print self.person
        if len(self.object) > 0:
            print "***** objects: "
            print self.object
        if len(self.fromLocation) > 0:
            print "***** from location: "
            print self.fromLocation
        if len(self.toLocation) > 0:
            print "***** to location: "
            print self.toLocation
        if len(self.location) > 0:
            print "***** locations: "
            print self.location
        #if len(self.furniture) > 0:
        #    print "furniture: "
        #    print self.furniture
        if len(self.reference) > 0:
            print "***** references: "
            print self.reference
        if len(self.locationModifier) > 0:
            print "***** loc modifiers: "
            print self.locationModifier

    def printme_final(self):
        #prints all fields in the objective that have values in them
        # and are required by Tiago to perform the defined Action.

        print "\n***** Split ACTION *****:  "+str(self.instance)
        print self.sentence
        print "**** Text to speak"
        print self.confirmationtext
        print("**** Action Type: ")
        print  self.comtype
        print "***** command: "
        print self.command
        if len(self.person) > 0:
            print "***** people: "
            print self.person
        if len(self.object) > 0:
            print "***** objects: "
            print self.object
        if len(self.fromLocation) > 0:
            print "***** from location: "
            print self.fromLocation
        if len(self.toLocation) > 0:
            print "***** to location: "
            print self.toLocation
 
        return

    def execute(self, actionnum):
        prt.info("\n***** EXECUTE for ACTION No: "+str(actionnum))
        if   self.comtype[0] == 's':
            prt.info("*** action is: SEARCHING")
            self.search()
        elif self.comtype[0] == 'm':
            prt.info("*** action is: MANIPULATING")
            self.get()
        elif self.comtype[0] == 'a': 
            prt.info("*** action is: ACCOMPANY")
            self.accompany()  

        return

    #######################    
    ###### SEARCHING ######
    def search(self):
        prt.info("***** in Execute SEARCH Section*****")
        found = False #todo should be False for final program!!
        for i in range(0,len(self.fromLocation)):
            frmLoc = self.fromLocation[i]
            frmcoords = self.getfoundloc(frmLoc)
            prt.info("in search: FROM loc  = "+frmLoc)  
            prt.info("in Search: From coords:"+str(frmcoords))
            ##### OBJECT type SEARCH
            if len(self.object) >0:
                #todo
                obj    = self.object[0]
           
                prt.info("in search: find object: "+ obj)
                self.tbm3ctrler.say("I am looking for object "+ obj +" on the "+frmLoc)
                ntrys = 5
                rc = self.tbm3ctrler.move_robot_to_coords(frmcoords,ntrys)

                prt.todo("remove forcing logic for ojbect & replace with OBJECT SEARCHING code")
                if i == 1:
                    found = True

                if found:
                    found = True
                    prt.info("Returned coords from object search code are:"+str(coords))
                    prt.info("in search: store coords of found location for object")

                    self.storefoundloc(obj, coords)
                    self.tbm3ctrler.say("I have found the "+obj+" at tlocation "+frmLoc)
                    break

            ##### PERSON type SEARCH
            if len(self.person) >0:  
                #todo
                per =  self.person[0]
                coords = self.getfoundloc(frmLoc)
                prt.info("in search: person loc= "+frmLoc)
                prt.info("in search: find person: "+ per)
                ntrys = 5
                rcfrm = self.tbm3ctrler.move_robot_to_coords(frmcoords,ntrys)
                self.tbm3ctrler.say("I am looking for a person called  "+ per +" in the "+frmLoc)

                prt.todo("remove forcing logic for person & replace with PERSON SEARCHING code")
                if i == 1:
                    found = True

                if found:
                    found = True  
                    prt.info(str("Returned coords from object search code are:"+str(frmcoords)))
                    prt.info("in search: store coords of found location for person")
                    self.storefoundloc(per, coords)

                    prt.info(str("Found coords for "+per+" search are:"+str(frmcoords)))
                    self.tbm3ctrler.say("I have found "+per+" in the"+frmLoc)
                    break

        #todo store status   
        if not found:
            prt.error("in search: !!!!! 'FROM' Location NOT FOUND !!!!!")   

        prt.todo("in search: store status of task")           
        prt.info("in search: ALL DONE")

        return        

    def storefoundloc(self,object,coords):
        #print("in search: object for dict coords :" + object )
        Objective.founditems[object] = coords

        return


    def getfoundloc(self, key):
        if Objective.founditems.has_key(key):
            coords = Objective.founditems[key]
        else:
            coords = []

        return coords
    #######################
    ###### MANIPULATE #####
    def get(self):
        prt.info("***** in Execute MANIPULATE section *****")
        #check if we have previously located the object
        obj = self.object[0]
        prt.debug(("in get: obj= "+obj))

        if Objective.founditems.has_key(obj):
            coords = Objective.founditems[obj]
            prt.info("in get: FROM location coords for: " + obj)
            prt.info(str(coords))
        else:
            # use FROM field 
            frmLoc = self.fromLocation[0]
            prt.debug("in get: frmLoc: "+ frmLoc)

            if Objective.founditems.has_key(frmLoc):

                # prt.debug("******* obj: "+obj)
                # prt.debug("*************************founditems")
                # for KEY in Objective.founditems:
                #     print("key: "+KEY+"--- "+str(Objective.founditems[KEY]))
                # prt.debug("*************************")
                frmcoords = Objective.founditems[frmLoc]
                prt.info("in get: use FROM location to find coords: " + frmLoc)
                prt.info(str(frmcoords))
            else:
                prt.error("in get: !!!!! no FROM location available !!!!!")
                return # can not proceed


        prt.todo("sortout pickup  logic with user interaction")
        pickupOK = True
        prt.info("in get: ########## Navigate to the FROM coords")
        prt.info("in get: pick up the: "+ obj)

        prt.info("in get: TO location is : "+ self.toLocation[0])
        toLoc = self.toLocation[0]
        prt.info("in get: toLoc : "+toLoc)
        if Objective.founditems.has_key(toLoc):
            tocoords = Objective.founditems[toLoc]
        
            prt.info("in get: coords for TO location for "+obj+" at "+toLoc )
            prt.info(str(tocoords))
        else:
            prt.error("in get: cannot find TO coords for obj: "+"TO is: " + toLoc)


        if pickupOK == True:
            #todo 
            prt.info("in get: goto FROM location"+frmLoc)
            self.tbm3ctrler.say("I am going to the "+frmLoc)
            ntrys = 5
            rcfrm = self.tbm3ctrler.move_robot_to_coords(frmcoords,ntrys) 
            self.tbm3ctrler.say("I have arrived at the "+frmLoc)

            self.tbm3ctrler.say("I now need to pick up the "+obj)


            prt.todo("in get: request that object taken from Tiago")
            self.tbm3ctrler.say("please take the "+obj+ "from me" )
            ntrys = 5
            rcto  =  self.tbm3ctrler.move_robot_to_coords(tocoords,ntrys)
            self.tbm3ctrler.say("I have arrived at the "+toLoc+" with the " +obj)

        else:
            prt.warning("in get: goto TO without object:" + self.object[0])   
            self.tbm3ctrler.say("now going to "+toLoc+" without the "+obj ) 
            rcfrm =  self.tbm3ctrler.move_robot_to_coords(tocoords)
            self.tbm3ctrler.say("I have arrived at the "+toLoc+" without the " +obj)
    
 

        #todo store status
        prt.todo("in get: store status of task")    
        prt.todo("in get: ALL DONE")

        return

    ##### ACCOMPANY
    def accompany(self):
        prt.info("***** in Execute ACCOMPANY section*****")
        #check that we have previously located the person
        per = self.person[0]

        if Objective.founditems.has_key(per):
            frmcoords = Objective.founditems[per]
            prt.info("in get: coords for person " + per)
            prt.info(str(frmcoords))
        else:
            prt.error("in accompany: cannot find coords for: " + per + " so check FROM loc")
        
            # if location of person not previously known    
            if self.fromLocation:
                #todo
                prt.info("in accompany: FROM Location defined")
                frmLoc  = self.fromLocation
                lenfrom = len(frmLoc)
                if lenfrom == 1:
                    prt.info("in accompany: use FROM location as a single value ie loc is known is: "+str(frmLoc))
                else:              
                #todo abort if FROM not available
                    prt.error("in accompany: FROM not available so abort task")

        ###################################################################################       
        prt.debug("command is: "+str(self.command[0])) 

        if   self.command[0] == 'guide':
            #todo
            toLoc = self.toLocation[0]
            prt.info("toLoc is: "+toLoc)
            if Objective.founditems.has_key(toLoc):
                tocoords = Objective.founditems[toLoc]
                self.tbm3ctrler.say("I am moving to meet "+per)
                ntry = 5
                rcto   = self.tbm3ctrler.move_robot_to_coords(frmcoords,ntry)
                self.tbm3ctrler.say(per+"Hello "+per+" please follow me to the "+toLoc)
                prt.info("in accompany: Say follow  me " +"to the "+toLoc)
                prt.info("in accompany: TO location is " + toLoc)
                ntry = 5
                rc   = self.tbm3ctrler.move_robot_to_coords(tocoords,ntry)

                #update persons location to their new location
                self.storefoundloc(per, toLoc)
       
                prt.todo("in accompany: for guiding -- allow for user too far behind tiago??")


        elif self.brlcommand[0] == 'follow':           
            #todo
            prt.info("in accomapny: Say Ready to follow you " + per)
            self.tbm3ctrler.say("Hello "+per+" please lead on and I will follow ")
            prt.todo("in accompany: listen for stop  command from user")

        #todo store status
        prt.info("in accompany: store status of task")    
        prt.info("in accompany: ALL DONE")

        return

    # def move_robot_to_coords(self,coords):
    #     # indirection code to allow deveopment with no robot attached 
    #     prt.todo("in robot_move_to: ADD CODE -if needed to reformat coordsfor pose2D??" )
    #     if self.IROBOT:
    #         prt.warning("ROBOT moving to : "+str(coords))
    #         self.move_to_pose2D(coords)
    #     else:
    #         prt.warning("NO ROBOT available for software to control!")   
    #         prt.warning("ROBOT will NOT moving to : "+str(coords)) 
    #     return

##### end of objective class defn  #####

class Analysis(object):
    def __init__(self,tbm3ctrler):
        self.tbm3ctrler = tbm3ctrler

        self.ERL_data =None
        self.locations =None
        self.commands=None
        self.people=None
        self.objects=None
        self.references      = ['him', 'her', 'it', 'them' ]

        self.locModifiers    = [ 'from', 'to', 'in']
        
    def process_task(self,task):
        # punctuation, the word 'and', converting to
        # lower case and adding leading and trailing spaces

        taskP = task.translate(None, string.punctuation)
        nd    = re.compile('(\s*)and(\s*)')
        taskP = nd.sub(' ', taskP)

        me    = re.compile('(\s*)me(\s*)')
        taskP = me.sub(' user ', taskP)

        my    = re.compile('(\s*)my(\s*)')
        taskP = my.sub(' user ', taskP)

        taskP = taskP.lower()
        taskP = ' ' + taskP + ' '

        return taskP

    def objectify(self,taskP):
        #take processed task text and generate a list of Objective classes
        coms       = []
        comstype   = []
        brlcoms    = []
        taskflags  = []
        objectives = []
        self.commandcount = 0
        
        #iterate through task looking for any commands, store them and their indexes in relevant list
        start = 0
        ptr   = 0
        for word in taskP.split():
            #test = ' ' + word + ' '
            test = word

            for cmd  in self.commands:       

                if cmd[1] ==  word:
                    self.commandcount += 1
                    comstype.append(cmd[0])
                    brlcoms.append(cmd[2])
                    coms.append(word) 
                    prt.debug("taskP[start:]"+taskP[start:]+str(start))

                    index = taskP[start:].find(test)
                    if index >-1:
                        taskflags.append(index+start-1)
                        start = taskflags[ptr]
                        ptr  += 1

        prt.info("Objectify Command count = "+str(self.commandcount))       
        if self.commandcount == 3:         
            #use the indices of commands to split task into objectives
            objective1 = Objective(taskP[taskflags[0]+1:taskflags[1]], coms[0],brlcoms[0],comstype[0],self, self.tbm3ctrler)
            objective2 = Objective(taskP[taskflags[1]+1:taskflags[2]], coms[1],brlcoms[1],comstype[1],self, self.tbm3ctrler)
            objective3 = Objective(taskP[taskflags[2]+1:len(taskP)]  , coms[2],brlcoms[2],comstype[2],self, self.tbm3ctrler)
            #DAR
            # print("taskflags[0]+1 :"+str(taskflags[0]+1))
            # print("taskflags[1]+1 :"+str(taskflags[1]+1))
            # print("taskflags[2]+1 :"+str(taskflags[2]+1))
            # print("len(taskP)     :"+str(len(taskP)))
            return [objective1,objective2,objective3]
        else:
            return []

        

    def resolveReferences(self,objectives):
        #if an objective references an object or person mentioned earlier, look at the previous objective and copy object or person data into current objective
         for i in range(1,len(objectives)):
            while len(objectives[i].reference) > 0:
                ref = objectives[i].reference[0]
                if ref == "him" or ref == "her" or ref == "them":
                    try:
                        objectives[i].person = objectives[i-1].person
                        del objectives[i].reference[0]
                    except IndexError:
                        print "Failed to resolve reference for '" + ref + "'"
                        del objectives[i].reference[0]
                elif ref == "it":
                    # if an 'object is defined in current action then don't do this 
                    try: 
                        objectives[i].object = objectives[i-1].object
                        del objectives[i].reference[0]
                    except IndexError:
                        print "Failed to resolve reference for '" + ref + "'"
                        del objectives[i].reference[0]
                else:
                    print "Unknown reference, how did this happen?  ref="+ref
                    del objectives[i].reference[0]

    ##### DAR routines follow ##########################################################################
    ####################################################################################################
    def read_ERL_data(self,filein):
        # read the ERL object/person versus pronbable locations table 
        # ERL Rule book (5 June 2018) page 11 Section 3.3.2

        with open(filein,'r') as csvfile:
            data = csvfile.readlines()

        obj_per_and_loc_list =[]
        for row in data:
                if '#' in row :
                    pass
                elif len(row.strip()) == 0:
                    pass
                else:
                    row = row.lower()
                    OType,obj_per,loc1, pc1, loc2, pc2, loc3, pc3 = row.split(',')

                    obj_per_and_loc_list.append(
                        [OType.strip(),  obj_per.strip(),            
                        [loc1.strip(),   int(pc1.strip())], 
                        [loc2.strip(),   int(pc2.strip())], 
                        [loc3.strip(),   int(pc3.strip())] ] )

        return obj_per_and_loc_list

    ####################################################################################################
    def read_ERL_verb(self, filein):
        # read the ERL commands (verbs) with brl equivalents
        # also preeceded by verl type:
        #   a = accompanying
        #   m = manipulating
        #   s = searching
        # ERL Rule book (5 June 2018) page 11 Section 3.3.2
        with open(filein,'r') as csvfile:
            data = csvfile.readlines()

        erl_brl_verb_list =[]
        for row in data:
                if '#' in row :
                    pass
                elif len(row.strip()) == 0:
                    pass
                else:
                    row = row.lower()

                    verb_type, ERL_verb, BRL_verb = row.split(',')

                    erl_brl_verb_list.append(
                        [verb_type.strip(),             
                         ERL_verb.strip(),   
                         BRL_verb.strip() ] )
       
        return erl_brl_verb_list

    #*********************************************************************************
    def get_obj_per_loc(self,obj_per): 
        # returns a list of the 3 possible locations for the person/object
        # without the % probability as not using this currently.

        # remove '_' for multi word object/person descriptors
        obj_per = obj_per.replace('_',' ')
        #locations =[]

        for row in self.ERL_data:
            if obj_per == row[1]:
                # locations.append(row[2][0]) 
                # locations.append(row[3][0]) 
                # locations.append(row[4][0]) 
                loc0 = row[2][0]
                loc1 = row[3][0] 
                loc2 = row[4][0]
                break

        return loc0,loc1,loc2

    #*********************************************************************************
    def parse_ERL_data(self):
        ### build list of people and a list of objects from
        ### the competition data table
        self.people  = []
        self.objects = []
        for line in self.ERL_data:
            word = line[1]
            if line[0] == 'i': # inanimate object
                #make multi word object a single string
                word = word.replace(' ','_')
                self.objects.append(word)

            elif line[0] == 'p': # a persons name

                self.people.append(word)

            else:
                prt.error('Objects file: not a I or P flag! see file: '+ objects_file)

    #*********************************************************************************
    def parse_locations(self):
        ### build a unique list of locations that are pertient to TBM3
        ### we need this list to identify the location as received in text from stt package.
        ### eg 'coffe table' is a 2 word location or 'kitchen cabinet'
        ### So we substitute '_' for any spaces. This then allows the 'parse' 
        ### routine to work with whole word loations.  
        locs_dict = {}
        locs_list = []

        for line in self.ERL_data:

            for i in range( 2, len(line)):
                #print( line[i][0])
                locs_dict[line[i][0]] = ''

        # build locations list from unique keys in dctionary        
        for key in locs_dict:
            #print("key: "+key)
            locs_list.append(key)

        # find any space delimited location and replace the ' ' with '_'    
        idx = -1
        for item in locs_list:
            idx += 1
            if ' ' in item:
                item=item.replace(' ','_')
                locs_list[idx]=item

        return locs_list

    #*********************************************************************************
    def process_locations(self,expr):
        # do in string search for all known locations
        for loc in self.locations:
            loc_no_ = loc.replace('_',' ')
            #print('loc_no_ :'+loc_no_)
            if  loc_no_ in expr:
                expr = expr.replace(loc_no_, loc)

        return expr 

    #*********************************************************************************
    def process_objects(self,expr):
        # do in string search for all known objects
        for obj in self.objects:
            obj_no_ = obj.replace('_',' ')
            if  obj_no_ in expr:
                expr = expr.replace(obj_no_, obj)


        return expr 

    def check_locations(self,navjson_file):


        #from pprint import pprint

        with open(navjson_file) as fh:
            data = json.load(fh)

        #pprint (data)

        # get top level keys - ie the location names from the map file
        keylist = data.keys()
        keylist.sort()

        # for each unique ERL loction checkit against the map file
        found  = 0
        missed = 0
        for item in self.locations:

            loc_no_ = item.replace('_',' ')
            if loc_no_ in keylist:
                found += 1
                prt.result(loc_no_)   
            else:
                missed += 1
                prt.error(loc_no_)

        return (found,missed)
    #*********************************************************************************
    def getcompdata(self):
        #todo put in launch file
        ERL_objects_file = rospy.get_param("TBM3_objects_file")
        ERL_verbs_file   = rospy.get_param("TBM3_verbs_file")

        self.ERL_data        = self.read_ERL_data(ERL_objects_file)

        # for line in ERL_data:
        #     print (line[0],line[1],line[2],line[3],line[4])

        self.commands        = self.read_ERL_verb(ERL_verbs_file)

        self.parse_ERL_data()

        self.locations       = self.parse_locations() # note 2 or more word locations returned with "_" instead of " "
        #for loc  in self.locations:
            #print(loc)
        self.checklocations()


    def checklocations(self):
        #check that all "ERL competition locations" found are in the Navigation locations.json file
        jsonfilein = rospy.get_param("locations_json")
        prt.todo("add json file exisits check in def language_v2.py")
        found,missed = self.check_locations(jsonfilein)
        prt.result("\nERL Locations checked against our map file\n - found: "+str(found)+" - Missed: "+str(missed)+"\n")
        # for cmd in commands:
        #     print(cmd)
        # for per in people:
        #     print("Person  : "+per)
        # for obj in  objects:
        #     print("Object  : "+obj)
        # for loc in locations:
            # print("Location: "+loc)


    #
    #================================================================================================
    #

    def defineobjectives(self,speech):
        taskP = self.process_task(speech)

        #create objectives from task text
        objectives = self.objectify(taskP)
        if self.commandcount == 3:
            #use task text to fill relevant variables in each objective
            objectives[0].parse()
            objectives[1].parse()
            objectives[2].parse()

            #attempt to change reference words (him, her etc) into the names of the objects/people they are referencing
            self.resolveReferences(objectives)

            # map ERL verb to BRL equivalent
            #     this also deals with "take" which can be manipulating OR accompanying 
            # deal with locations of known objects (no location verbally given) eg Find John
            # 
            objectives[0].process_objective()
            objectives[1].process_objective()
            objectives[2].process_objective()

        return self.commandcount, objectives

    def getconfirmationtext(self,objectives):
        acttxt_0 = objectives[0].confirmationtext
        acttxt_1 = objectives[1].confirmationtext
        acttxt_2 = objectives[2].confirmationtext

        action_txt =  acttxt_0 +' then ' + acttxt_1 + ' and '+ acttxt_2 +'\n'
        print("\n Text to be spoken to Granny Annie by TiaGO ..........")
        print (action_txt)

        return action_txt

    def executeobjectives(self,objectives):
        ### then pass data on to the actions(x3) driver!!
        objectives[0].execute(1)
        objectives[1].execute(2)
        objectives[2].execute(3)

if __name__ == '__main__':
    pass
'''
    #hard coded example commands - first four from ERL documentation
    task1 = "Locate Tracy, lead her to the bedroom, and bring me an apple from the kitchen cabinet."

    task2 = "Locate the bottle, place it in the bucket, and take John from the bedroom to the exit."

    task3 = "Take  me to the bedroom, find my glasses, and bring me a pear from the kitchen table."

    task4 = "Give me the cup on the coffee table, find John, and follow him."

    task5 = "Hi Tiago, please will you go and find John, I need you to take him to the exit and then get me a bottle of water from the kitchen cabinet, thanks!"

    task6 = "Search for the tea pot, Get my glasses, and Bring me a pear from the coffee table."

    task7 = "Find for john , Get my glasses, and Bring me a pear from the coffee table."

    task8 = "find the pear, get me the pear and find for my glasses"

    task = task1
    print('\n***** Orignal task words *****')
    print(task)
    print '\n***** Actions remapped to BRL versions/plus other remaps'

    analysis = Analysis()

    analysis.getcompdata()

    theobjectives = analysis.defineobjectives(task1)

    ans = analysis.getconfirmationtext(theobjectives)

    analysis.executeobectives(theobjectives)

    #simplify text ease of use
    # taskP = process_task(task)

    # #create objectives from task text
    # objectives = objectify(taskP)

    # #use task text to fill relevant variables in each objective
    # objectives[0].parse()
    # objectives[1].parse()
    # objectives[2].parse()

    # #attempt to change reference words (him, her etc) into the names of the objects/people they are referencing
    # resolveReferences(objectives)

    # #print objectives for user to read
    # # objectives[0].printme()
    # # objectives[1].printme()
    # # objectives[2].printme()

    # # map ERL verb to BRL equivalent
    # #     this also deals with "take" which can be manipulating OR accompanying 
    # # deal with locations of known objects (no location verbally given) eg Find John
    # # 
    # objectives[0].process_objective()
    # objectives[1].process_objective()
    # objectives[2].process_objective()


    # #print objectives for user to read
    # objectives[0].printme_final()
    # objectives[1].printme_final()
    # objectives[2].printme_final()

    # acttxt_0 = objectives[0].confirmationtext
    # acttxt_1 = objectives[1].confirmationtext
    # acttxt_2 = objectives[2].confirmationtext

    # action_txt = "*** "+ acttxt_0 +' then ' + acttxt_1 + ' and '+ acttxt_2 +'\n'
    # print("\n Text to be spoken to Granny Annie by TiaGO ..........")
    # print (action_txt)

    # ### then pass data on to the actions(x3) driver!!
    # objectives[0].execute()
    # objectives[1].execute()
    # objectives[2].execute()
'''
