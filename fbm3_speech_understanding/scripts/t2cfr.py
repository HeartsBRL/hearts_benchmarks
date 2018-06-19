#!/usr/bin/python

# Filename: T2CFR.py
# Created: ?? May 2017
# Author : Zeke Steer 
# Purpose: Take a text string from "Speech to Text" and use TextRazor to
#          convert to CFR format.  
################################################################################
# Updates:
#
# 23 Nov 2017 Deek  - Implement "Python Support Library" ROS package  
#                     First shared module is tag_topics.py
#
# 07 Aug 2017 Derek - Make competion data path seperate from the one used to test
#                     the code using runmode = TH eg test harness.
#
# 11 Jul 2017 Derek - Removed debug/dev print statements
#
# 26 Jun 2017 Derek - add results files for mic input as needed by ERL competition
#         
################################################################################
import rospy 
from std_msgs.msg import String 
import operator
import random
import textrazor
import python_support_library.tag_topics  as TT
import os

o_tt=TT.tag_topics()
run_mode        = rospy.get_param('SR_TH')
ERLDATAPATHOUT  = rospy.get_param("SR_ERL_DATAPATHOUT")

class Command:

    def __init__(self):
        self.verb = None
        self.action = None
        self.theme = None
        self.conjunction = None
        self.goal = None
        self.source = None
        self.ground = None
        self.beneficiary = None
        self.path = None
    
    def __str__(self):
        s = self.action + "("
        has_args = False
        if not self.beneficiary is None:
            s += "beneficiary:\"%s\"," %self.beneficiary
            has_args = True
        if not self.theme is None:
            s += "theme:\"%s\"," %self.theme
            has_args = True
        if not self.goal is None:
            s += "goal:\"%s\"," %self.goal
            has_args = True
        if not self.source is None:
            s += "source:\"%s\"," %self.source
            has_args = True
        if not self.ground is None:
            s += "ground:\"%s\"," %self.ground
            has_args = True
        if not self.path is None:
            s += "path:\"%s\"," %self.path
            has_args = True
        if has_args:    
            s = s[:len(s) - 1]
        s += ")"
        return s

def get_root(words):
    for word in words:
        if word.parent is None:
            return word
    return None

def get_goal_source_theme(parent, tokens):
    n_tokens = len(tokens)

    if (parent.relation_to_parent == "nn" or parent.relation_to_parent == "amod") and n_tokens > 0:
        tokens.insert(n_tokens - 1, parent.token)
    else:
        # add proposition e.g. to, of, etc.
        if not parent.parent is None and parent.relation_to_parent == "pobj" and (n_tokens == 0 or parent.parent.token != tokens[n_tokens - 1]):
            #if parent.parent.parent is None or parent.parent.parent.relation_to_parent != "ccomp":
            if parent.parent.parent is None or parent.parent.parent.relation_to_parent != "ccomp" or parent.parent.token == "for":
                tokens.append(parent.parent.token)

        # add determiner e.g. the, my, etc.
        for child in parent.children:
            if child.relation_to_parent == "det" or child.relation_to_parent == "poss":
                if len(child.children) > 0:
                    for grandchild in child.children:
                        # combine any possessives
                        if grandchild.relation_to_parent == "possessive":
                            tokens.append(child.token + grandchild.token)
                            break
                        tokens.append(child.token)
                        break
                else:
                    tokens.append(child.token)

        tokens.append(parent.token)

    for child in parent.children:
        if child.relation_to_parent == "pobj" or child.relation_to_parent == "prep" or child.relation_to_parent == "nn" or child.relation_to_parent == "amod":
             get_goal_source_theme(child, tokens)

def get_goal_source_theme_excluded(parent, tokens, exclude):
    n_tokens = len(tokens)

    if (parent.relation_to_parent == "nn" or parent.relation_to_parent == "amod") and n_tokens > 0:
        tokens.insert(n_tokens - 1, parent.token)
    else:
        # add proposition e.g. to, of, etc.
        if not parent.parent is None and parent.relation_to_parent == "pobj" and (n_tokens == 0 or parent.parent.token != tokens[n_tokens - 1]):
            #if parent.parent.parent is None or parent.parent.parent.relation_to_parent != "ccomp":
            if parent.parent.parent is None or parent.parent.parent.relation_to_parent != "ccomp" or parent.parent.token == "for":
                tokens.append(parent.parent.token)

        # add determiner e.g. the, my, etc.
        for child in parent.children:
            if child.relation_to_parent == "det" or child.relation_to_parent == "poss":
                if len(child.children) > 0:
                    for grandchild in child.children:
                        # combine any possessives
                        if grandchild.relation_to_parent == "possessive":
                            tokens.append(child.token + grandchild.token)
                            break
                        tokens.append(child.token)
                        break
                else:
                    tokens.append(child.token)

        tokens.append(parent.token)

    for child in parent.children:
        if child.relation_to_parent == "pobj" or child.relation_to_parent == "prep" or child.relation_to_parent == "nn" or child.relation_to_parent == "amod"  or child.relation_to_parent == "poss":
             if not child is exclude:
                 get_goal_source_theme(child, tokens)

def get_arg(parent):
    tokens = [ ]
    get_goal_source_theme(parent, tokens)
    return ' '.join(tokens)

def get_arg_excluded(parent, exclude):
    tokens = [ ]
    get_goal_source_theme_excluded(parent, tokens, exclude)
    return ' '.join(tokens)

# should only generate bringing goal if this method returns false, otherwise should append to theme
def get_prep(parent):
    if parent.relation_to_parent == "prep":
        return parent

    for child in parent.children:
        prep = get_prep(child)
        if not prep is None:
            return prep
    
    return None

def get_child_prep(parent):
    for child in parent.children:
        prep = get_prep(child)
        if not prep is None:
            return prep

    return None

def set_cmd(child, cmd):
    if child.relation_to_parent == "dobj" or child.relation_to_parent == "pobj": 
        if cmd.action == "MOTION":
            cmd.goal = get_arg(child)  
        elif cmd.action == "BRINGING":
            cmd.theme = get_arg(child)
        elif cmd.action == "SEARCHING":
            cmd.theme = get_arg(child)
        elif cmd.action == "PLACING":
            cmd.theme = get_arg(child)
        elif cmd.action == "TAKING":
            cmd.theme = get_arg(child)
    elif child.relation_to_parent == "cc":
        cmd.conjunction = child.token
    elif child.relation_to_parent == "prep": 
        if len(child.children) > 0:
            if cmd.action == "MOTION":
                if child.token == "in" or child.token == "into" or child.token == "to" or child.token == "near":
                    if cmd.goal is None: 
                        cmd.goal = get_arg(child.children[0])
                    else:
                        cmd.goal = cmd.goal + " " + get_arg(child.children[0])
                else:
                    if child.token == "from":
                        cmd.path = child.token + " " + get_arg(child.children[0])
                    else:
                        cmd.path = get_arg(child.children[0])
            elif cmd.action == "BRINGING":
                if child.token == "from":
                    cmd.source = get_arg(child.children[0])
                else:
                    child_prep = get_child_prep(child)
                    if child_prep is None:
                        # e.g. "onto"
                        cmd.goal = get_arg(child.children[0])
                    else:
                        cmd.theme = cmd.theme + " " + get_arg_excluded(child.children[0], child_prep)
                        set_cmd(child_prep, cmd)
            elif cmd.action == "SEARCHING":
                if child.token == "for":
                    cmd.theme = get_arg(child.children[0])
                else:
                    cmd.ground = get_arg(child.children[0])
            elif cmd.action == "PLACING":
                cmd.goal = get_arg(child.children[0])
            elif cmd.action == "TAKING":
                if child.token == "with":
                    cmd.theme = cmd.theme + " " + get_arg(child.children[0])
                else:
                    cmd.source = get_arg(child.children[0])
        else:
            if cmd.action == "MOTION":
                if child.token == "around":
                    cmd.path = child.token
    elif child.relation_to_parent == "iobj":
        cmd.beneficiary = child.token
    elif child.relation_to_parent == "acomp":
        if (len(child.children) > 0) and child.token == "next":
            cmd.goal = "next " + get_arg(child.children[0])
    elif child.relation_to_parent == "expl":
        if child.token == "there":
            cmd.goal = child.token
    elif child.relation_to_parent == "advmod":
        if (len(child.children) > 0) and (child.token == "next" or child.token == "away"):
            cmd.goal = child.token + " " + get_arg(child.children[0])
    elif child.relation_to_parent == "nsubj":
        if child.token == "it":
            cmd.theme = child.token

placing = 0

def get_cmds(verb, actions_by_verb, cmds):
    global placing
     
    if verb.token == "go" and verb.parent != None and verb.parent.token == "let":
        action = ''
        placing = 1
    elif placing == 1 and verb.parent != None and verb.parent.token == "go":
        action = "PLACING"
        vb = "let go"
        placing = 0
    elif verb.token in actions_by_verb:
        action = actions_by_verb[verb.token]
        placing = 0
        vb = verb.token
    else:
        action = ''
        placing = 0

    cmd = None

    if action != '':
        cmd = Command()
        cmd.action = action
        cmd.verb = vb
        cmds.append(cmd)
        
    for child in verb.children:
        if cmd != None and child.token == "close":
            cmd.goal = get_arg(child)
        elif cmd != None and child.token == "please" and (len(child.children) == 1):
            set_cmd(child.children[0], cmd)
        elif placing or child.token in actions_by_verb.keys():
            get_cmds(child, actions_by_verb, cmds)       
        elif cmd != None:
            set_cmd(child, cmd)
                    
def get_actions_by_verb():
    actions_by_verb = { }
    VERBSFILE=rospy.get_param('SR_VERBSFILE')
    with open(VERBSFILE, "r") as file:
        for line in file.readlines():
            tokens = line.strip().split(",") 
            actions_by_verb[tokens[0]] = tokens[1]
    return actions_by_verb

def get_cfr(s):

    response = client.analyze(s)
  
    actions_by_verb = get_actions_by_verb()
    root = get_root(response.words())
    cmds = [ ]
    get_cmds(root, actions_by_verb, cmds)

    indexes_by_cmd = { }
    for cmd in cmds:
        index = 0
        while (index != -1):
            index = s.find(cmd.verb, index)
            if index in indexes_by_cmd.values():
                index = index + 1
            else:
                indexes_by_cmd[cmd] = index
                break
    sorted_cmds = sorted(indexes_by_cmd.items(), key=operator.itemgetter(1))

    cmd_strs = [ ]
    for cmd in sorted_cmds:
        cmd_strs.append(str(cmd[0]))

    cfr = "#".join(cmd_strs)
    return cfr

def get_actions_by_verb():
    actions_by_verb = { }
    VERBSFILE=rospy.get_param('SR_VERBSFILE')
    with open(VERBSFILE, "r") as file:
        for line in file.readlines():
            tokens = line.strip().split(",") 
            actions_by_verb[tokens[0]] = tokens[1]
    return actions_by_verb

pub_topic = 'CFR_Out'
sub_topic = "/hearts/stt"

# zeke's key:
#textrazor.api_key = "9a7bb531e0ea81b43e8fba17a517152d4d016847ab88a63a54c6cb36"
# alex's key:
textrazor.api_key = "211ef8b5891adff67b329277425597a000d96bd443eeca0fe977819e"
client = textrazor.TextRazor(extractors=["entities", "dependency-trees", "phrases", "words"])

#cfr = get_cfr(str1)
#print("CFR = "+cfr)
pub = rospy.Publisher(pub_topic, String, queue_size=10) 

rospy.init_node('cfr_node', anonymous=True)

def writeresults(WAV,S2T,CFR):
    resultsfile = ERLDATAPATHOUT+"results.txt"
    # ERL only want raw "file name" on USB stick
    wavfile = os.path.basename(WAV)
    fh = open (resultsfile,'a')
    str = wavfile+'|'+S2T.strip()+'|'+CFR+'\n'
    fh.write(str)
    fh.close() 

def callback(s):
 
    rospy.loginfo(rospy.get_name()+": Received text message: %s"%s.data)
    #TOPIC is in format <Test from speech string> ~ <wav file name> 
    str1, str2 = s.data.split('~')

    if(run_mode == 'TH'): 
        # strip off #index# prefix used by TH mode 
        index, str1 = o_tt.get_key(str1)

    print("*** T2CFR.py - calling    msg.data = get_cfr(str1)\n*** to derive CFR syntax from interpreted Text")
    msg = String()
    msg.data = get_cfr(str1)
  
 # check the CFR command has arguments ie NOT Motion()
 #            OR not zero length
    if "()" not in msg.data:        
       if len(msg.data) > 0 : 
           pass
       else:
           msg.data = 'NO_INTERPRETATION'
    else:
        msg.data = 'NO_INTERPRETATION'

    if(run_mode == 'TH'): 
        # add back #index# prefix used by TH mode 
        rtndata = o_tt.add_key(index,msg.data)
    else: 
        rtndata = msg.data
 
    pub.publish(rtndata)
    rospy.loginfo(rospy.get_name()+": Published CFR message: %s"%msg.data+'\nmaybe changed to: '+rtndata)
    
    # ERL formatted results file
    writeresults(str2,str1,msg.data)  

rospy.Subscriber(sub_topic, String, callback) 
rospy.spin()
