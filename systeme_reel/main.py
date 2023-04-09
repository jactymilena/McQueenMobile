from SunFounder_Line_Follower import Line_Follower
from SunFounder_Ultrasonic_Avoidance import Ultrasonic_Avoidance
from picar import front_wheels
from picar import back_wheels
import time
import picar
import constants as const

import numpy as np
import math


class Car:
    def __init__(self):
        self.ua = Ultrasonic_Avoidance.Ultrasonic_Avoidance(20)
        self.lf = Line_Follower.Line_Follower()
        self.fw = front_wheels.Front_Wheels(db='config')
        self.bw = back_wheels.Back_Wheels(db='config')
        self.fw.turning_max = 45
        self.speed = 0.0
        self.is_dec = False
        self.is_acc = False
        self.start_time_acc = 0
        
    
    def move_by_dist(self, dist, backwards=False):
        self.fw.turn_straight()
        sec = dist/(const.FORWARD_SPEED*const.SPEED_RATE/100.0)
       
        self.move(sec, backwards, acc=False)
        

    def move(self, sec=0, backwards=False, acc=True, decelerate=False):
        # self.fw.turn_straight()
        if acc:
            self.init_acceleration(decelerate)
        else:
            self.bw.speed = const.FORWARD_SPEED

        if backwards:
            self.bw.backward()
        else:
            self.bw.forward()
        time.sleep(sec)
        
    
    def turn_right(self, sec=2):
        self.fw.turn_right()
        self.bw.forward()
        self.bw.speed = const.FORWARD_SPEED
        time.sleep(sec)


    def turn_left(self, sec=2): 
        self.fw.turn_left()
        self.bw.forward()
        self.bw.speed = const.FORWARD_SPEED
        time.sleep(sec)


    def turn(self, angle):
        self.is_acc = False
        self.fw.turn(angle)
        self.bw.forward()
        # self.bw.speed = const.FORWARD_SPEED
        time.sleep(1)


    def stop(self, sec=0):
        self.is_acc = False
        self.bw.stop()
        self.fw.turn_straight()
        time.sleep(sec)


    def obstacle_avoidance(self):
        self.turn_right()
        self.move_by_dist(5)
        self.turn_left()
        self.move_by_dist(40)
        self.turn_left()
        self.move_by_dist(40)
        self.turn_right()



            
    def accelerate(self):
        self.speed = self.speed + (const.ACC_RATE * const.ACC_TIME) * (-1 if self.is_dec else 1)

    # def accelerate_by_dist(dist):

    
    
    def init_acceleration(self, decelerate=False):
        if decelerate:
            if not self.is_dec:
                self.is_dec = True
                self.is_acc = False
                self.start_time_acc = time.time()
        else:
            if not self.is_acc:
                self.is_acc = True
                self.is_dec = False
                self.start_time_acc = time.time()
        
    
    def apply_speed(self):
        percentage_speed = (self.speed/const.SPEED_RATE)*100.0
        if percentage_speed > 100.0:
            self.bw.speed = 100
        elif percentage_speed < 0:
            self.bw.speed = 0
        else:
            self.bw.speed = int(percentage_speed)
        
        
    def check_acceleration(self):
        can_acc = False
        if self.is_dec:
            can_acc = self.speed > 0
        else:
            self.is_dec = False
            
        if self.is_acc:
            can_acc = self.speed < const.SPEED_RATE
        else:
            self.is_acc = False
        
        print("can_acc " + str(can_acc))
        
        if can_acc:
            # print("time.time()" + str(time.time()) + " start_time_acc " + str(self.start_time_acc))
            if np.abs(time.time() - self.start_time_acc) >= const.ACC_TIME:
                self.start_time_acc = time.time()
                return True
        
        return False     
        
        
    def run(self):
        count = 0
        self.fw.turn_straight()
        # self.ua.calibrate()
        while True:
            print("acc " + str(self.is_acc) + " speed " + str(self.speed))
            if self.ua.detect_obstacle():
                print("OBSTACLE DETECTED")
                self.stop(2)
                self.move_by_dist(20, backwards=True)
                # self.move(1, backwards=True, acc=False)
                self.stop(1)
                self.obstacle_avoidance()
                # #time.sleep(1)
                # self.ua.clear_measures()
                # break
                
            else:
                self.move(backwards=False, acc=True,decelerate=False)

            
            turning_angle = self.lf.follow_line(const.LINE_STEP)
            
            if(turning_angle == -1):
                # Trajectory end
                self.stop()
                break
            else:
                self.fw.turn(turning_angle)
            

            if self.check_acceleration():
                self.accelerate()
                self.apply_speed()

            time.sleep(0.2)
            count += 1

            # if count > 300:
            #     self.stop()
            #     # Trajectory end (TODO add line follower check for end)
            #     break


    

    def test(self):

        # self.fw.turn_straight()
        # time.sleep(5)
        # self.fw.turn(90) 

        # threshold = 10

        # while True:
        #     distance = self.ua.get_distance()
        #     status = self.ua.less_than(threshold)
        #     if distance != -1:
        #         print('distance', distance, 'cm')
        #         time.sleep(0.2)
        #     else:
        #         print(False)
        #     if status == 1:
        #         print("Less than %d" % threshold)
        #     elif status == 0:
        #         print("Over %d" % threshold)
        #     else:
        #         print("Read distance error.")

        # self.move_by_dist(10)
        self.obstacle_avoidance()
        self.stop()

        # self.fw.turn_right()
        # self.bw.forward()
        # self.bw.speed = const.FORWARD_SPEED
        # # time.sleep(2.5)
        # # self.fw.turn_straight()
        # time.sleep(2)
        # self.stop()

        # turning_angle = 90
        # # self.fw.turn(turning_angle)
        # self.fw.turn_right()
        # self.bw.forward()
        # self.bw.speed = const.FORWARD_SPEED
        # time.sleep(5)

        # while True:
            # self.move(2)

            # if self.check_acceleration():
            #     self.accelerate()
            #     self.apply_speed()
        # self.stop()


def main():
    picar.setup()
    time.sleep(5)
    c = Car()
    try:
        # c.test()
        c.run()
    except KeyboardInterrupt:
        c.stop()


if __name__ == '__main__':
    main()