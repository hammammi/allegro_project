#!/usr/bin/env python3.8

import rospy
import serial
import struct
import numpy as np
import threading
from allegro_multi_finger_msgs.msg import Contact
from matplotlib import pyplot as plt
from numpy import savetxt
from time import time


# present_tactile1_data = 0
# present_tactile2_data = 0
prev_tactile1_data = 0
prev_tactile2_data = 0
contact_sensor = Contact()

finger1 = 0
finger2 = 0

ser = serial.Serial(port = '/dev/ttyUSB0', baudrate=2000000)

def thread_run():
    global contact_sensor, pub
    pub.publish(contact_sensor)

    # rospy.loginfo(value)
    threading.Timer(0.01,thread_run).start()


def talker():
    global contact_sensor, pub
    rospy.init_node('contact_sensor',anonymous=True)
    pub = rospy.Publisher('contact_data',Contact, queue_size=1)
    data = np.zeros(20)
    finger1 = np.empty((0,6))
    finger2 = np.empty((0,6))

    ser.reset_input_buffer()

    thread_run()
    rospy.loginfo("started")

    while not rospy.is_shutdown():
        response = ser.readline()
        # rospy.loginfo(response.__len__())
        # rospy.loginfo(response)
        # rospy.loginfo(response[0])
        if response.__len__() == 12 + 2:
            # rospy.loginfo(data.__len__())
            temp = rospy.Time.now()
            # s = temp.secs
            # t = float(temp.secs)+float(temp.nsecs*1e-9)
            data = np.array(struct.unpack('<13B', response[:-1]))
            rospy.loginfo('')
            if data[0] == 97:
                # rospy.loginfo(data3[1:5])
                # rospy.loginfo(data3[5:9])
                contact_sensor.finger1[0:7] = data[1:7]
                contact_sensor.finger2[0:7] = data[7:13]
                finger1 = np.append(finger1,[data[1:7]],axis=0)
                finger2 = np.append(finger2,[data[7:13]],axis=0)

                rospy.loginfo(contact_sensor.finger1)
                rospy.loginfo(contact_sensor.finger2)
                # rospy.loginfo(contact_sensor.finger1[8:])
                # rospy.loginfo(contact_sensor.finger2[8:])
                # if present_tactile1_data > 6 or present_tactile1_data < -0.1:
                #     present_tactile1_data = prev_tactile1_data
                # prev_tactile1_data = present_tactile1_data
                # contact_f.tactile1 = present_tactile1_data
            ser.reset_input_buffer()

    savetxt('./hard3.csv',finger1,delimiter=',')
    savetxt('./soft3.csv',finger2,delimiter=',')

    # rospy.loginfo(finger1)
    # plt.subplot(2,3,1)
    fig1 = plt.figure(1)
    ax1 = fig1.subplots()
    ax1.plot(finger1[:,0],label = 'senser1')
    # plt.subplot(2,3,2)
    ax1.plot(finger1[:,1],label = 'senser2')
    # plt.subplot(2,3,3)
    ax1.plot(finger1[:,2],label = 'senser3')
    # plt.subplot(2,3,4)
    ax1.plot(finger1[:,3],label = 'senser4')
    # plt.subplot(2,3,5)
    ax1.plot(finger1[:,4],label = 'senser5')
    # plt.subplot(2,3,6)
    ax1.plot(finger1[:,5],label = 'senser6')
    ax1.legend()
    # plt.plot(finger1)
    fig1.savefig('./hard3.png')
    plt.show()

    fig2 = plt.figure(2)
    ax2 = fig2.subplots()
    # plt.subplot(2,3,1)
    ax2.plot(finger2[:,0],label = 'senser1')
    # plt.subplot(2,3,2)
    ax2.plot(finger2[:,1],label = 'senser2')
    # plt.subplot(2,3,3)
    ax2.plot(finger2[:,2],label = 'senser3')
    # plt.subplot(2,3,4)
    ax2.plot(finger2[:,3],label = 'senser4')
    # plt.subplot(2,3,5)
    ax2.plot(finger2[:,4],label = 'senser5')
    # plt.subplot(2,3,6)
    ax2.plot(finger2[:,5],label = 'senser6')
    ax2.legend()
    fig2.savefig('./soft3.png')
    plt.show()



if __name__ == '__main__':
    try :
        talker()
    except rospy.ROSInterruptException:
        # ser1.close()
        # ser2.close()


        ser.close()

        pass