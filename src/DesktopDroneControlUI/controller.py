#!/usr/bin/env python
import roslib; roslib.load_manifest('rviz_python_tutorial');
# roslib.load_manifest('gazebo')
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
# from gazebo.srv import GetModelState
from tf.transformations import euler_from_quaternion
from math import atan, degrees, sqrt

import rospy
from std_msgs.msg import String

RADIAL_CONST = 850

class Controller():
    """ Complex motion controller for the drone. """
    R_TO_L = 1
    L_TO_R = -1

    def __init__(self):
        """A constructor."""
        self.actions = {
                'Circle'    : self.circle,
                'Emergency' : self.emergency,
                'Takeoff'   : self.passFunc,
                'Land'  : self.passFunc,
                'D_TO_UP'   : self.moveForward,
                'UP_TO_D'   : self.moveBackward,
                'L_TO_R'    : self.moveLeft,
                'R_TO_L'    : self.moveRight,
                'ZIGZAG'    : self.zigzag,
                'RZIGZAG'   : self.rzigzag,
                'LToR'  : self.lToR,
                'RToL'  : self.rToL,
                'Test1'  : self.test1,
                'Test2'  : self.test2,
                'Test3'  : self.test3,
                'Move'  : self.move,
                'Repeat': self.repeat
        }
        self.history = [0, 0, 0]
        self.action = ''
        self.ptam_data = ''
        rospy.init_node('drone_controller', anonymous=True)
        rospy.Subscriber("drone_ctrl_directions", String, self.callback)
        rospy.Subscriber("chatter", String, self.handle_ptam_init)
        # rospy.Subscriber("ptam/mappoints", String, self.handle_mappoints)

        self.publisher = rospy.Publisher('cmd_vel', Twist)
        self.publisher2 = rospy.Publisher('meneldor/ui_com', String)
        self.tum_ardrone_pub = rospy.Publisher('tum_ardrone/com', String)

        self.radi = 0

    def emergency(self, args):
        pass

    def passFunc(self, args):
        pass

    def land(self, args):
        pass

    def test1(self, args):
        
        self.radi = 0

        # bot_orientation, gms, resp1 = self.find_bot_orientation()
        # print resp1.pose.position.x, resp1.pose.position.y, resp1.pose.position.z

        self.tum_ardrone_pub.publish(String("p space"))
        for i in xrange(100):
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.5
            self.publisher.publish(twist)
            rospy.sleep(0.01)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher.publish(twist)
        self.tum_ardrone_pub.publish(String("p space"))

        # bot_orientation, gms, resp1 = self.find_bot_orientation()
        # print resp1.pose.position.x, resp1.pose.position.y, resp1.pose.position.z

        rospy.sleep(0.50)

    def test2(self, args):
        # bot_orientation, gms, resp1 = self.find_bot_orientation()
        # print resp1.pose.position.x, resp1.pose.position.y, resp1.pose.position.z

        self.tum_ardrone_pub.publish(String("p reset"))
        rospy.sleep(0.50)

        self.tum_ardrone_pub.publish(String("p space"))
        for i in xrange(100):
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = -0.3
            self.publisher.publish(twist)
            rospy.sleep(0.01)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher.publish(twist)
        self.tum_ardrone_pub.publish(String("p space"))

        # bot_orientation, gms, resp1 = self.find_bot_orientation()
        # print resp1.pose.position.x, resp1.pose.position.y, resp1.pose.position.z

        rospy.sleep(1.50)

    def test3(self, args):
        print "calling circle with radius ", self.radi/2
        return
        rospy.sleep(4.00)
        for i in xrange(70):
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = -0.4
            twist.linear.z = 0.0
            self.publisher.publish(twist)
            rospy.sleep(0.01)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher.publish(twist)
        rospy.sleep(0.50)
        for i in xrange(70):
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.4
            twist.linear.z = 0.0
            self.publisher.publish(twist)
            rospy.sleep(0.01)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher.publish(twist)
        rospy.sleep(0.50)
        for i in xrange(50):
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 1.0
            self.publisher.publish(twist)
            rospy.sleep(0.01)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher.publish(twist)
        rospy.sleep(0.50)
        for i in xrange(50):
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = -1.0
            self.publisher.publish(twist)
            rospy.sleep(0.01)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher.publish(twist)

        # self.simple_circle(self.radi/2)

    def move(self, args):
        print args
        move_distance_x = float(args[0])
        move_distance_y = float(args[1])

        # Right handed system. So, negative velocity
        y_vel = -move_distance_y/move_distance_x

        for i in xrange(int(round(100 * move_distance_x))):
            twist = Twist()
            twist.linear.x = 1.0
            twist.linear.y = y_vel
            self.publisher.publish(twist)
            rospy.sleep(0.01)
            twist.linear.y = 0.0
            twist.linear.x = 0.0
            self.publisher.publish(twist)

    def test_old(self, args):
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        print resp1.pose.position.x, resp1.pose.position.y
        twist = Twist()
        twist.linear.y = 1.00
        self.publisher.publish(twist)
        rospy.sleep(1.0)
        twist.linear.y = 0.0
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        print resp1.pose.position.x, resp1.pose.position.y
        twist = Twist()
        twist.linear.y = 2.00
        self.publisher.publish(twist)
        rospy.sleep(1.0)
        twist.linear.y = 0.0
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        print resp1.pose.position.x, resp1.pose.position.y
        twist = Twist()
        twist.linear.x = 1.00
        self.publisher.publish(twist)
        rospy.sleep(1.0)
        twist.linear.y = 0.0
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        print resp1.pose.position.x, resp1.pose.position.y

    def precompute(self, args):
        """Common code to initialize motion."""
        target_x = float(args[0])
        target_y = float(args[1])
        self.history[0] = target_x
        self.history[1] = target_y

        # history[2] corresponds to current location
        # to be used later on for repeat.
        self.history[2] = 0
        return [target_x, target_y]

    def moveLinear(self, x_vel, y_vel):
        for i in xrange(100):
            twist = Twist()
            twist.linear.x = x_vel
            twist.linear.y = y_vel
            self.publisher.publish(twist)
            rospy.sleep(0.01)
            twist.linear.y = 0.0
            twist.linear.x = 0.0
            self.publisher.publish(twist)

    def moveForward(self, args):
        print 'mf'
        self.moveLinear(1, 0)

    def moveBackward(self, args):
        print 'mb'
        self.moveLinear(-1, 0)

    def moveLeft(self, args):
        print 'ml'
        self.moveLinear(0, 1)

    def moveRight(self, args):
        print 'mr'
        self.moveLinear(0, -1)

    def lToR(self, args):
        """Function to move drone from left to right."""
        target_x, target_y = self.precompute(args)
        param_list = [8, 7, 6, 5]

        self.circleObject(target_x, target_y,
                          param_list, False, -1)
        self.history[2] = 4
        self.publisher2.publish(String("Snap " + str(4)))

    def rToL(self, args):
        target_x, target_y = self.precompute(args)
        param_list = [4, 5, 6, 7]

        self.circleObject(target_x, target_y,
                          param_list, False, 1)
        self.history[2] = 0
        self.publisher2.publish(String("Snap " + str(0)))

    def repeat(self, args):
        if self.action == 'circle':
            print "Position chosen", args[0]
            dest = int(args[0])
            param = dest - self.history[2]
            direction = 1
            if param < 0:
                param += 8
            if param > 4:
                direction = -1
                param = 8 - param
            self.circleObject(self.history[0],
                              self.history[1], 
                              xrange(param),
                              True, direction)
            self.history[2] = dest

        elif self.action in ['zigzag', 'rzigzag']:
            print "Z Position chosen", args[0]
            dest = int(args[0])
            v_dist = (dest - self.history[2])/2
            z_dir = (dest-self.history[2])/abs(dest-self.history[2])
            y_dir = 0
            if (dest - self.history[2])%2 == 1:
                if dest%2 == 0:
                    y_dir = 1
                else:
                    y_dir = -1

            v_dir = 0
            if self.action == 'zigzag':
                v_dir = -1
            if self.action == 'rzigzag':
                v_dir = 1

            twist = Twist()
            print v_dir, y_dir, v_dist
            for i in xrange(abs(v_dist)):
                twist.linear.z = v_dir*0.505*z_dir
                for j in xrange(300):
                    self.publisher.publish(twist)
                    rospy.sleep(0.01)
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                self.publisher.publish(twist)

            if y_dir != 0:
                for j in xrange(300):
                    twist.linear.y = 1.085*y_dir
                    self.publisher.publish(twist)
                    rospy.sleep(0.01)
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                self.publisher.publish(twist)

            self.history[2] = dest

    def zigzag(self, args):
        # -1 is for down
        self.action = 'zigzag'
        self.doZigzag(-1)

    def rzigzag(self, args):
        # 1 is for up
        self.action = 'rzigzag'
        self.doZigzag(1)

    def doZigzag(self, v_dir):
        self.history[2] = (5 - (v_dir*5))/2

        # May be depending on the distance from the 
        # object, the zigzag length can be determined.
        print "Doing zigzag"
        mult = -1
        zmult = 0
        rospy.sleep(0.1)

        self.publisher2.publish(String("Snap " + str(5 - self.history[2])))

        numbers = [300, 300, 300, 300, 300]
        for i in xrange(5):
            twist = Twist()
            twist.linear.y = -v_dir*0.05*mult
            twist.linear.z = v_dir*0.15*zmult
            zmult = (zmult+1)%2
            for j in xrange(numbers[i]):
                self.publisher.publish(twist)
                rospy.sleep(0.01)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher.publish(twist)
            mult *= -1
            rospy.sleep(0.5)
            self.publisher2.publish(String("Snap " + \
                    str( abs( -((v_dir*5) + 5)/2 + i+1))))
            print i
        print "done"

    def circle(self, args):
        self.action = 'circle'
        target_x, target_y = self.precompute(args)
        self.circleObject(target_x, target_y, xrange(7+1), False, 1)

    def circleObject(self, target_x, target_y, param_list, is_repeat,
                     direction):

        # Depending on distance from target, move circularly for 
        # appropriate time.
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        orig_radius = sqrt((target_x - resp1.pose.position.x)**2 +
                           (target_y - resp1.pose.position.y)**2)

        print "Taking a circle of radius", orig_radius
        orig_yaw = bot_orientation[2]

        prev_yaw = orig_yaw
        self.turn_towards_ROI(target_x, target_y)
        rospy.sleep(1.4)
        for i in param_list:
            yaw_diff = 0
            if not is_repeat:
                self.publisher2.publish(String("Snap " + str(i%8)))
            while abs(yaw_diff) < (3.14/4):
                twist = Twist()
                twist.linear.y = direction * 1.35

                # NOTE: This value must depend on the radius of
                # the circle.
                twist.linear.x = 0.285
                self.publisher.publish(twist)
                rospy.sleep(0.004)
                twist.linear.y = 0.0
                twist.linear.x = 0.0
                self.publisher.publish(twist)
                yaw = self.turn_towards_ROI(target_x, target_y)

                yaw_diff = yaw - prev_yaw
                if yaw_diff > 5.3:
                    yaw_diff -= 6.28
                if yaw_diff < -5.3:
                    yaw_diff += 6.28
                
            bot_orientation, gms, resp1 = self.find_bot_orientation()
            radius = sqrt((target_x - resp1.pose.position.x)**2 +
                               (target_y - resp1.pose.position.y)**2)
            yaw = bot_orientation[2]

            prev_yaw = yaw
            print "radius", radius
            print "yaw", yaw

        bot_orientation, gms, resp1 = self.find_bot_orientation()
        radius = sqrt((target_x - resp1.pose.position.x)**2 +
                           (target_y - resp1.pose.position.y)**2)

        print "Final radius", radius

    def callback(self, data):
        print rospy.get_name() + ": I heard %s" % data.data
        ctrl_command = data.data.split()
        self.actions[ctrl_command[0]](ctrl_command[1:])
        return
        target_x = float(ctrl_direction[2])
        target_y = float(ctrl_direction[3])

    def listen(self):
        print "Started listening"
        rospy.spin()

    def find_bot_orientation(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        bot_orientation = [-1, -1, -1]
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp1 = gms('quadrotor', 'world')
            # print resp1.pose
            orientation = resp1.pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            bot_orientation = euler_from_quaternion(quaternion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        return [bot_orientation, gms, resp1]

    def turn_towards_ROI(self, target_x, target_y):
        # Find position of your bot.
        bot_orientation, gms, resp1 = self.find_bot_orientation()

        # Find the direction you are facing
        # print "bot orientation is ", bot_orientation
        yaw = bot_orientation[2]
        if yaw < 0:
            yaw += 6.28

        # Find the direction to turn to
        target_orientation = -1
        if resp1.pose.position.x == target_x:
            if resp1.pose.position.y > target_y:
                target_orientation = 3.14/2.0
            else:
                target_orientation = -3.14/2.0
        else:
            target_orientation = atan( (target_y-resp1.pose.position.y)/(target_x-resp1.pose.position.x) )
            if target_orientation < 0 and (target_y > resp1.pose.position.y):
                target_orientation += 3.14
            elif target_orientation < 0 and (target_y < resp1.pose.position.y):
                target_orientation += 6.28
            elif target_orientation > 0 and (target_y < resp1.pose.position.y):
                target_orientation += 3.14

        # print "Target Details"
        # print target_orientation
        # print degrees(target_orientation)
        # print 'my yaw = ', yaw

        # Turn towards the object.
        while abs(yaw - target_orientation) > 0.1:
            twist = Twist()
            factor1 = -1 if abs(yaw - target_orientation) > 3.14 else 1
            factor2 = -1 if yaw > target_orientation else 1

            factor2 *= factor1

            twist.angular.z = 1.2 * factor2
            self.publisher.publish(twist)

            rospy.sleep(0.04)

            twist.angular.z = 0.0
            self.publisher.publish(twist)
            try:
                resp1 = gms('quadrotor', 'world')
                orientation = resp1.pose.orientation
                quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
                bot_orientation = euler_from_quaternion(quaternion)
                yaw = bot_orientation[2]
                if yaw < 0:
                    yaw += 6.28
                # print "Yaw is", yaw, "and target is", target_orientation
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        
        return yaw

    def handle_ptam_init(self, data):
        split_data = data.data.split('\n')

        # Need to properly decide the format. 
        # Currently, it is all random, really.
        split_data.pop(0) # data: -----------
        l = split_data.pop(0) # Matches: XX
        if int(l.split()[1]) < 20:
            print "Oh, my! Very few matches"

        l = split_data.pop(0) # dTransMagn: XX
        print "dTransMagn ", l

        l = split_data.pop(0) # dTransIMUMagn: XX
        print "dTransIMUMagn ", l

        radius = 8 * float(split_data[-5].split(':')[1])
        print "radius ", radius

        self.radi += radius

        print "Chatter got"
        # self.simple_circle2(0.2)

    def handle_mappoints(self, data):
        split_data = data.data.split('\n')

        points = []
        for line in split_data:
            numbers = line.split()
            if len(numbers) > 2:
                points.append([float(numbers[0]), float(numbers[1]), float(numbers[2])])
        
        points.sort(key= lambda x: x[2])
        print split_data[:2]
        print points

    def simple_circle(self, radius):

        if radius > 12:
            print "Radius too large?"
            return

        for i in xrange(500):
            twist = Twist()
            twist.linear.y = radius * -0.16
            twist.angular.z = 0.20
            self.publisher.publish(twist)
            rospy.sleep(0.004)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

    def simple_circle2(self, radius):

        if radius > 12:
            print "Radius too large?"
            return

        for i in xrange(500):
            twist = Twist()
            twist.linear.y = radius * -0.16
            twist.angular.z = 0.40
            self.publisher.publish(twist)
            rospy.sleep(0.004)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)


if __name__ == '__main__':
    controller = Controller()
    controller.listen()
