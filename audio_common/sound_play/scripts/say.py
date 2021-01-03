#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from std_msgs.msg import Float64

voice  = 'voice_kal_diphone'
volume = 1.0
rospy.init_node('SoundControl', anonymous=True)
pub_ = rospy.Publisher("CommandMattern", String, queue_size=10)
pub_1 = rospy.Publisher("cmd_robotarm", Float64, queue_size=10)
pub_2 = rospy.Publisher("/darknet_ros/CommandMattern", Float64, queue_size=10)
soundhandle = SoundClient()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    string = data.data
    if (string == 'Turn up.'):
        soundhandle.say('OK.', voice, volume)
        pub_1.publish(5.0)
    elif (string == '跟我走。'):
        soundhandle.say('OK.', voice, volume)
        pub_2.publish(1.0)
    elif (string == 'Turn down.'):
        soundhandle.say('OK.', voice, volume)
        pub_1.publish(6.0)
    elif (string == 'Turn left.'):
        soundhandle.say('OK.', voice, volume)
        pub_1.publish(3.0)
    elif (string == 'Turn right.'):
        soundhandle.say('OK.', voice, volume)
        pub_1.publish(4.0)
    elif (string == 'Forward.'):
        soundhandle.say('OK.', voice, volume)
        pub_1.publish(1.0)
    elif (string == 'Back.'):
        soundhandle.say('OK.', voice, volume)
        pub_1.publish(2.0)
    elif (string == '小杜。'):
        soundhandle.say('Yeah?', voice, volume)
    elif (string == 'Happy new year.'):
        soundhandle.say('Happy new year!', voice, volume)
    else:
        pass
class SoundControl:
    def __init__(self):
        
        
        rospy.Subscriber("voiceWords", String, callback)
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin() 
        
        rospy.sleep(1)
  
    

if __name__ == '__main__':
    
    soundcontrol = SoundControl()
    # rospy.loginfo('Saying: %s' % s)
    # rospy.loginfo('Voice: %s' % voice)
    # rospy.loginfo('Volume: %s' % volume)

    #soundhandle.say(s, voice, volume)
