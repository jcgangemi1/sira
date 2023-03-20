#!/usr/bin/env python3

# ROS
import rospy

# Speech Recognition
import speech_recognition as sr
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# Messages
from std_msgs.msg import Float32, Int8

#Triggers
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class ColorListener():
    def __init__(self):
        rospy.init_node('color_listener')

        # Config
        self.rate = 10

    def main(self):
        recording_length = 10 # seconds

        r = sr.Recognizer()

        # rospy.wait_for_service('/grasp_object/trigger_grasp_object')
        # rospy.loginfo('Node connected to /grasp_object/trigger_grasp_object.')
        # trigger_grasp_object_service = rospy.ServiceProxy('/grasp_object/trigger_grasp_object', Trigger)

        with sr.Microphone() as source:
            # record
            rospy.loginfo('RECORDING')
            audio_clip = r.record(source, duration=recording_length)
            
            # recognize
            try:
                text_string = r.recognize_google(audio_clip)  # see PyPI docs for other options
                rospy.loginfo('RECOGNIZING')
            except sr.UnknownValueError:
                print("Speech recognizer could not understand audio")
                text_string = 0
            except sr.RequestError as e:
                print("Speech recognition error; {0}".format(e))
                text_string = 0

            rospy.loginfo('text string is: ' + str(text_string))

            # execute
            if text_string == 0:
                rospy.loginfo("NONE")
                soundhandle = SoundClient()
                rospy.sleep(1)
                soundhandle.say(text="I did not hear anything.", voice = 'voice_kal_diphone', volume = 1.0)
                rospy.sleep(1)
            elif "hey" in text_string:
                rospy.loginfo("HOT WORD")
                if "Sarah" or "Sira" in text_string:
                    rospy.loginfo("CYRA")
                    soundhandle = SoundClient()
                    rospy.sleep(1)
                    soundhandle.say(text="Yes, what do you need Julia?", voice = 'voice_kal_diphone', volume = 1.0)
                    rospy.sleep(1)

                    audio_clip = r.record(source, duration=recording_length)
                    # recognize
                    try:
                        text_string = r.recognize_google(audio_clip)  # see PyPI docs for other options
                        rospy.loginfo('RECOGNIZING')
                    except sr.UnknownValueError:
                        print("Speech recognizer could not understand audio")
                        text_string = 0
                        soundhandle = SoundClient()
                        rospy.sleep(1)
                        soundhandle.say(text="Sorry, I did not understand that!", voice = 'voice_kal_diphone', volume = 1.0)
                        rospy.sleep(1)
                    except sr.RequestError as e:
                        print("Speech recognition error; {0}".format(e))
                        text_string = 0

                    if text_string != 0:
                        if "pick" or "grab" or "grass" or "grasp" in text_string:
                            soundhandle = SoundClient()
                            rospy.loginfo('GRASP')
                            rospy.sleep(1)
                            soundhandle.say(text="Yes, I can grab that for you!", voice = 'voice_kal_diphone', volume = 1.0)
                            rospy.sleep(1)
                            rospy.loginfo('INITIATING GRASP DEMO')
                            trigger_request = TriggerRequest() 
                            trigger_result = trigger_grasp_object_service(trigger_request)
                            print('trigger_result = {0}'.format(trigger_result))
            else:
                rospy.loginfo("NO HOT WORD")
                soundhandle = SoundClient()
                rospy.sleep(1)
                soundhandle.say(text="Sorry, I do not understand what you said.", voice = 'voice_kal_diphone', volume = 1.0)
                rospy.sleep(1)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    ColorListener().main()