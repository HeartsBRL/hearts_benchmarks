#!/usr/bin/env python

import numpy as np
import string
import re

class Objective:

    def __init__(self,sentence,com,brlcom,comtype):
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

    def parse(self):
        #reads sentence and stores key words

        #check for 2 (or more) word locations & objects
        self.sentence = process_locations(self.sentence)
        self.sentence = process_objects  (self.sentence)
        splitList = self.sentence.split()

        for word in splitList:
            test = word 
            
            if test in locations:
                self.location.append(word)
            #elif test in furnitures:
            #    self.furniture.append(word)
           
            elif test in objects:
                self.object.append(word)
            elif test in people:
                self.person.append(word)
            elif test in references:
                self.reference.append(word)
            elif test in locModifiers:
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
            elif word == "to":
                try:
                    self.toLocation.append(self.location[count])
                    del self.locationModifier[0]
                except IndexError:
                    #print "No location matching 'to' modifier"
                    del self.locationModifier[0]
            count = count + 1

    def process_objective(self):
        #ensure underscore removed from location tyep lists before data passed on to Tiago
        if len(self.location) > 0:
             self.location[0]  = self.location[0].replace('_',' ')
        if len(self.fromLocation) > 0:
             self.fromLocation[0]  = self.fromLocation[0].replace('_',' ')
        if len(self.toLocation) > 0:
             self.toLocation[0]  = self.toLocation[0].replace('_',' ')


        #  below self.command[0] & comtyope[0] will be changed to the
        #  BRL equivalent before compariosn takes place
        brl_com, com_type = self.get_brl_com(self.command[0])
        self.command[0] =  brl_com
        self.comtype[0] =  com_type

        ## SEARCH ## - when there is no "toLocation"  for 'find/get an object'
        if (brl_com == 'find' or brl_com == 'get') and len(self.object) > 0 and len(self.location) == 0:
            value = get_obj_per_loc(self.object[0], ERL_data)
            self.toLocation.append(value)

        ## SEARCH ## - when there is no "toLocation"  for 'find/get a  person'
        if (brl_com == 'find' or brl_com == 'get') and len(self.person) > 0 and len(self.location) == 0:
            value = get_obj_per_loc(self.person[0], ERL_data)
            self.toLocation.append(value)   

        return

    def get_brl_com(self, erl_com):    
        # find the equivalent BRL command to the ERL one
        for cmd in commands:
            if erl_com == cmd[1]:
                brl_com = cmd[2]
                comtype = cmd[0]
                break
        #************************
        # **** Special case *****
        #************************
        # "take " can either be a "MANIPULATING" or "ACOMPANYING" command.
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

        print "\n***** Split ACTION *****: "
        print self.sentence
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


def process_task(task):
    # process the task, removing punctuation, the word 'and', converting to
    # lower case and adding leading and trailing spaces

    taskP = task.translate(None, string.punctuation)
    nd    = re.compile('(\s*)and(\s*)')
    taskP = nd.sub(' ', taskP)
    me    = re.compile('(\s*)me(\s*)')
    taskP = me.sub(' user ', taskP)
    taskP = taskP.lower()
    taskP = ' ' + taskP + ' '

    return taskP

def objectify(taskP):
    #take processed task text and generate a list of Objective classes
    coms       = []
    comstype   = []
    brlcoms    = []
    taskflags  = []
    objectives = []
    
    #iterate through task looking for any commands, store them and their indexes in relevant list
    for word in taskP.split():
        test = ' ' + word + ' '

        for cmd  in commands:

            if cmd[1] ==  word:
                comstype.append(cmd[0])
                brlcoms.append(cmd[2])
                coms.append(word) 

                index = taskP.find(test)
                if index >-1:
                    taskflags.append(index)

    #use the indices of commands to split task into objectives
    objective1 = Objective(taskP[taskflags[0]+1:taskflags[1]], coms[0],brlcoms[0],comstype[0])
    objective2 = Objective(taskP[taskflags[1]+1:taskflags[2]], coms[1],brlcoms[1],comstype[1])
    objective3 = Objective(taskP[taskflags[2]+1:len(taskP)]  , coms[2],brlcoms[2],comstype[2])

    return [objective1,objective2,objective3]

def resolveReferences(objectives):
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
def read_ERL_data(filein):
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
def read_ERL_verb(filein):
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
def get_obj_per_loc(obj_per,table): 
    # returns a list of the 3 possible locations for the person/object
    # without the % probability as not using this currently.

    # remove '_' for multi word object/person descriptors
    obj_per = obj_per.replace('_',' ')
    locations =[]

    for row in table:
        if obj_per == row[1]:
            locations.append(row[2][0]) 
            locations.append(row[3][0]) 
            locations.append(row[4][0]) 
            break

    return locations

#*********************************************************************************
def parse_ERL_data(table):
    ### build list of people and a list of objects from
    ### the competition data table
    people  = []
    objects = []
    for line in table:
        word = line[1]
        if line[0] == 'i': # inanimate object
            #make multi word object a single string
            word = word.replace(' ','_')
            objects.append(word)

        elif line[0] == 'p': # a persons name

            people.append(word)

        else:
            print('***** ERROR in file: not a I or P flag! see file: '+ objects_file)

    return people, objects

#*********************************************************************************
def parse_locations(table):
    ### build a unique list of locations that are pertient to TBM3
    ### we need this list to identify the location as received in text from stt package.
    ### eg 'coffe table' is a 2 word location or 'kitchen cabinet'
    ### So we substitute '_' for any spaces. This then allows the 'parse' 
    ### routine to work with whole word loations.  
    locs_dict = {}
    locs_list = []

    for line in table:

        for i in range( 2, len(line)):
            #print( line[i][0])
            locs_dict[line[i][0]] = ''

    # build locations list from unique keys in dctionary        
    for key in locs_dict:
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
def process_locations(expr):
    # do in string search for all known locations
    for loc in locations:
        loc_no_ = loc.replace('_',' ')
        #print('loc_no_ :'+loc_no_)
        if  loc_no_ in expr:
            expr = expr.replace(loc_no_, loc)

    return expr 

#*********************************************************************************
def process_objects(expr):
    # do in string search for all known objects
    for obj in objects:
        obj_no_ = obj.replace('_',' ')

        if  obj_no_ in expr:
            expr = expr.replace(obj_no_, obj)

    return expr 

#*********************************************************************************

ERL_data_file   = '../data/TBM3_objects.csv'
ERL_verb_file   = '../data/TBM3_verbs.csv'

ERL_data        = read_ERL_data(ERL_data_file)
commands        = read_ERL_verb(ERL_verb_file)

people, objects = parse_ERL_data(ERL_data)

locations       = parse_locations(ERL_data) # note 2 or more word locations returned with "_" instead of " "

references      = ['him', 'her', 'it', 'them' ]

locModifiers    = [ 'from', 'to']
#
#================================================================================================
#

if __name__ == '__main__':

    #hard coded example commands - first four from ERL documentation
    task1 = "Locate Tracy, lead her to the bedroom, and bring me an apple from the kitchen cabinet."

    task2 = "Locate the bottle, place it in the trash bin, and take John from the bedroom to the exit."

    task3 = "Take  me to the bedroom, find my glasses, and bring me a pear from the kitchen table."

    task4 = "Give me the cup on the coffee table, find John, and follow him."

    task5 = "Hi Tiago, please will you go and find John, I need you to take him to the exit and then get me a bottle of water from the kitchen cabinet, thanks!"

    task6 = "Search for the tea pot, Get my glasses, and Bring me a pear from the coffee table."

    task = task1

    print('\n***** Orignal task words *****')
    print(task)
    print '\n***** Actions remapped to BRL versions/plus other remaps'
    #simplify text ease of use
    taskP = process_task(task)

    #create objectives from task text
    objectives = objectify(taskP)

    #use task text to fill relevant variables in each objective
    objectives[0].parse()
    objectives[1].parse()
    objectives[2].parse()

    #attempt to change reference words (him, her etc) into the names of the objects/people they are referencing
    resolveReferences(objectives)

    #print objectives for user to read
    # objectives[0].printme()
    # objectives[1].printme()
    # objectives[2].printme()

    # map ERL verb to BRL equivalent
    #     this also deals with "take" which can be manipulating OR accompanying 
    # deal with locations of known objects (no location verbally given) eg Find John
    # 
    objectives[0].process_objective()
    objectives[1].process_objective()
    objectives[2].process_objective()


    #print objectives for user to read
    objectives[0].printme_final()
    objectives[1].printme_final()
    objectives[2].printme_final()

    ### at some point must build the "confirmation" text for all 3 actions.

    ### then pass data on to the actions(x3) driver!!