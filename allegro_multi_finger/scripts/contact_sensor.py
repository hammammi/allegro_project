#!/usr/bin/env python3.8

import rospy
import serial
import struct
import numpy as np
import threading
from allegro_multi_finger_msgs.msg import Contact

data = np.zeros(20)
value = 0
# present_tactile1_data = 0
# present_tactile2_data = 0
prev_tactile1_data = 0
prev_tactile2_data = 0
contact_sensor = Contact()

# ser1 = serial.Serial(port = '/dev/ttyUSB0', baudrate=2000000)
# ser2 = serial.Serial(port = '/dev/ttyUSB1', baudrate=2000000)
ser3 = serial.Serial(port = '/dev/ttyUSB0', baudrate=2000000)

def thread_run():
    global contact_sensor, pub
    pub.publish(contact_sensor)

    # rospy.loginfo(value)
    threading.Timer(0.01,thread_run).start()


def talker():
    global contact_sensor, pub
    rospy.init_node('contact_sensor',anonymous=True)
    pub = rospy.Publisher('contact_data',Contact, queue_size=1)

    # ser1.reset_input_buffer()
    # ser2.reset_input_buffer()
    ser3.reset_input_buffer()

    thread_run()
    rospy.loginfo("started")

    while not rospy.is_shutdown():
        # value = ser.read()
        # response1 = ser1.readline()
        # response2 = ser2.readline()
        response3 = ser3.readline()
        # rospy.loginfo(response1.__len__())
        # rospy.loginfo(response1)
        # rospy.loginfo(response1[0])
        # if response1.__len__() == 8 + 2:
        #     data1 = np.array(struct.unpack('<9B', response1[:-1]))
        #     # rospy.loginfo(data1)
        #     if data1[0] == 97 :
        #         rospy.loginfo(data1[1:])
        #         contact_sensor.finger1[0:8] = data1[1:]
        #         rospy.loginfo(contact_sensor.finger1[0:8])
        #         # if present_tactile1_data > 6 or present_tactile1_data < -0.1:
        #         #     present_tactile1_data = prev_tactile1_data
        #         # prev_tactile1_data = present_tactile1_data
        #         # contact_f.tactile1 = present_tactile1_data
        #         # rospy.loginfo(contact_f.tactile1)
        #                 # rospy.loginfo(response[12:14])
        #     # rospy.loginfo(value)
        #     ser1.reset_input_buffer()
        # if response2.__len__() == 8 + 2:
        #     data2 = np.array(struct.unpack('<9B', response2[:-1]))
        #     rospy.loginfo(data2)
        #     if data2[0] == 98 :
        #         rospy.loginfo(data2[1:])
        #         contact_sensor.finger2[0:8] = data2[1:]
        #         rospy.loginfo(contact_sensor.finger2[0:8])
        #         # if present_tactile1_data > 6 or present_tactile1_data < -0.1:
        #         #     present_tactile1_data = prev_tactile1_data
        #         # prev_tactile1_data = present_tactile1_data
        #         # contact_f.tactile1 = present_tactile1_data
        #         # rospy.loginfo(contact_f.tactile1)
        #                 # rospy.loginfo(response[12:14])
        #     # rospy.loginfo(value)
        #     ser2.reset_input_buffer()

        if response3.__len__() == 8 + 2:
            data3 = np.array(struct.unpack('<9B', response3[:-1]))
            # rospy.loginfo(data3)
            if data3[0] == 99:
                # rospy.loginfo(data3[1:5])
                # rospy.loginfo(data3[5:9])
                contact_sensor.finger1[8:] = data3[1:5]
                contact_sensor.finger2[8:] = data3[5:9]

                rospy.loginfo(contact_sensor.finger1[8:9])
                # rospy.loginfo(contact_sensor.finger1[8:])
                # rospy.loginfo(contact_sensor.finger2[8:])
                # if present_tactile1_data > 6 or present_tactile1_data < -0.1:
                #     present_tactile1_data = prev_tactile1_data
                # prev_tactile1_data = present_tactile1_data
                # contact_f.tactile1 = present_tactile1_data
                # rospy.loginfo(contact_f.tactile1)
                # rospy.loginfo(response[12:14])
            # rospy.loginfo(value)
            ser3.reset_input_buffer()


if __name__ == '__main__':
    try :
        talker()
    except rospy.ROSInterruptException:
        # ser1.close()
        # ser2.close()
        ser3.close()

        pass