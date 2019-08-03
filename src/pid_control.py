#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from random import randint
from math import sin
import time

class PID(object):

    def __init__(self, init_setPoint_value, init_state):

        self.setPoint_value = init_setPoint_value
        self.state_value = init_state

        self._setpoint_pub = rospy.Publisher("/setpoint",Float64,queue_size=1)
        self._state_pub = rospy.Publisher("/state",Float64,queue_size=1)
        self._check_all_publishers_ready()

        self._control_effort_sub = rospy.Subscriber('/control_effort', Float64, self.control_effort_callback)
        self._check_all_subscriptions_ready()
        self._control_effort_value = Float64()


    def _check_all_publishers_ready(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rospy.logdebug("START Publishers SENSORS READY CHECK")
        self._check_pub_connection(self._setpoint_pub)
        self._check_pub_connection(self._state_pub)
        rospy.logdebug("ALL Publishers READY")

    def _check_pub_connection(self, publisher_object):

        rate = rospy.Rate(10)  # 10hz
        while publisher_object.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug(
                "No susbribers to publisher_object yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("publisher_object Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def _check_all_subscriptions_ready(self):
        rospy.logdebug("START ALL Subscriptions READY")
        self._control_effort_ready()
        rospy.logdebug("ALL Subscriptions READY")

    def _control_effort_ready(self):
        self._control_effort_value = None
        rospy.logdebug("Waiting for /control_effort to be READY...")
        while self._control_effort_value is None and not rospy.is_shutdown():
            # We set the init_setPoint_value
            rospy.logdebug("Publishing Initial State and Setpoint...")
            rospy.logdebug("State="+str(self.state_value)+",SetValue="+str(self.setPoint_value))
            self.state_update(value=self.state_value)
            self.setpoint_update(value=self.setPoint_value)
            rospy.logdebug("Publishing Initial State and Setpoint...DONE")
            try:
                self._control_effort_value = rospy.wait_for_message(
                    '/control_effort', Float64, timeout=1.0)
                rospy.logdebug("Current /control_effort READY=>")

            except:
                rospy.logerr(
                    "Current /control_effort not ready yet, retrying for getting control_effort")


    def control_effort_callback(self,data):
        try:
            self._control_effort_value.data = data.data
        except AttributeError:
            rospy.logerr("control_effort has no value yet="+str(data.data))



    def setpoint_update(self, value):
        self.setPoint_value = value
        value_object = Float64()
        value_object.data = self.setPoint_value
        self._setpoint_pub.publish(value_object)

    def state_update(self, value):
        self.state_value = value
        value_object = Float64()
        value_object.data = self.state_value
        self._state_pub.publish(value_object)

    def get_control_effort(self):
        return self._control_effort_value.data



def sinus_test():
    rospy.init_node('sinus_pid_node', anonymous=True)


    setPoint_value = 0.0
    i = 0
    state_value = sin(i)
    pid_object = PID(init_setPoint_value = setPoint_value,
                        init_state = state_value)


    rate = rospy.Rate(15.0)
    ctrl_c = False
    def shutdownhook():
        rospy.loginfo("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    # We can change the set point if we want any time
    setPoint_value = 0.0
    pid_object.setpoint_update(value=setPoint_value)

    while not ctrl_c:
        state_value = sin(i)
        pid_object.state_update(value=state_value)
        effort_value = pid_object.get_control_effort()
        rospy.loginfo("state_value ==>"+str(state_value))
        rospy.loginfo("effort_value ==>"+str(effort_value))
        rate.sleep()
        i += 0.1

def step_test():
    rospy.init_node('step_pid_node', anonymous=True)

    setPoint_value = 0.0
    state_value = 1.0
    pid_object = PID(init_setPoint_value = setPoint_value,
                        init_state = state_value)


    rate = rospy.Rate(15.0)
    ctrl_c = False
    def shutdownhook():
        rospy.loginfo("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    # We can change the set point if we want any time
    setPoint_value = 0.0
    pid_object.setpoint_update(value=setPoint_value)

    i = 0


    while not ctrl_c:

        pid_object.state_update(value=state_value)
        effort_value = pid_object.get_control_effort()
        rospy.loginfo("state_value ==>"+str(state_value))
        rospy.loginfo("effort_value ==>"+str(effort_value))
        rate.sleep()
        i += 1
        if i > 30:
            state_value *= -1
            i = 0


if __name__ == '__main__':
    #step_test()
    sinus_test()