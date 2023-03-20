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

def _parse_string_for_unique_colors(text_string):
    rospy.loginfo("RAW STRING: %s" % text_string)
    words = text_string.split()
    unique_words = list(set(words))
    #unique_colors = []

    #for word in unique_words:
        #if any([word is color_string for color_string in colors]) or any(
                #[color_string in word for color_string in colors]):
            #unique_colors.append(word)

    return unique_words


class ColorListener():
    def __init__(self):
        rospy.init_node('color_listener')

        # Configure speech recognizer
        self.recognizer = sr.Recognizer()

        # Pub/Sub
        self.start_recording_subscriber = rospy.Subscriber(
            "/speech/start_recording", Float32, self.start_recording_callback
        )
        self.n_unique_colors_publisher = rospy.Publisher(
            "/speech/n_unique_colors", Int8, queue_size=1
        )

        # Config
        self.rate = 10

    def _predict_text(self, audio_clip):
        rospy.loginfo("Processing Audio...")
        try:
            # return self.recognizer.recognize_sphinx(audio_clip)
            return self.recognizer.recognize_google(audio_clip)
        except sr.UnknownValueError:
            rospy.loginfo("Speech recognizer could not understand audio")
            return None
        except sr.RequestError as e:
            rospy.loginfo("Speech recognition error; {0}".format(e))
            return None

    def start_recording_callback(self):
        recording_length = 10
        # TODO: make a while loop that records small chunks?
        with sr.Microphone() as source:
            audio_clip = self.recognizer.record(source, duration=recording_length)
            text_string = self._predict_text(audio_clip)
            rospy.loginfo("recognized text: {}".format(text_string))
            if text_string is not None:
                unique_colors = _parse_string_for_unique_colors(text_string)
                rospy.loginfo("detected colors {}".format(unique_colors))
                rospy.loginfo("*" * 40)
                rospy.loginfo("*" * 40)
                rospy.loginfo("number of colors {}".format(len(unique_colors)))
                rospy.loginfo("*" * 40)
                rospy.loginfo("*" * 40)
                self.n_unique_colors_publisher.publish(len(unique_colors))
            else:
                rospy.loginfo("Error speech not recognized!")

    def main(self):

        rospy.loginfo('TRYING NEW WAY')

        recording_length = 10 # seconds

        r = sr.Recognizer()

        rospy.wait_for_service('/grasp_object/trigger_grasp_object')
        rospy.loginfo('Node connected to /grasp_object/trigger_grasp_object.')
        trigger_grasp_object_service = rospy.ServiceProxy('/grasp_object/trigger_grasp_object', Trigger)

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

            # execute
            if text_string == 0:
                print("No command heard")
                soundhandle = SoundClient()
                rospy.sleep(1)
                soundhandle.say(text="I did not hear anything.", voice = 'voice_kal_diphone', volume = 1.0)
                rospy.sleep(1)
            elif "hey" in text_string:
                if "Sarah" or "Sira" in text_string:
                    rospy.loginfo('RESPONDING TO REQUEST')
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
                        if "pick" or "grab" or "grass" in text_string:
                            soundhandle = SoundClient()
                            rospy.sleep(1)
                            soundhandle.say(text="Yes, I can grab that for you!", voice = 'voice_kal_diphone', volume = 1.0)
                            rospy.sleep(1)
                            rospy.loginfo('INITIATING GRASP DEMO')
                            trigger_request = TriggerRequest() 
                            trigger_result = trigger_grasp_object_service(trigger_request)
                            print('trigger_result = {0}'.format(trigger_result))
            else:
                print("No command found in string")
                soundhandle = SoundClient()
                rospy.sleep(1)
                soundhandle.say(text="Sorry, I do not understand what you said.", voice = 'voice_kal_diphone', volume = 1.0)
                rospy.sleep(1)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    ColorListener().main()