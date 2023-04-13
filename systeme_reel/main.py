from SunFounder_Line_Follower import Line_Follower
from SunFounder_Ultrasonic_Avoidance import Ultrasonic_Avoidance
from picar import front_wheels
from picar import back_wheels
import time
import picar
import constants as const

import numpy as np


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
        self.turn_count = 0
        self.end_obs_avoidance = False
        
    
    def move_by_dist(self, dist, backwards=False):
        self.fw.turn_straight()
        sec = dist/(const.BACKWARD_SPEED*const.MAX_SPEED_RATE/100.0)
       
        self.move(sec, backwards, acc=False)
        

    def move(self, sec=0, backwards=False, acc=True, decelerate=False):
        if acc:
            self.init_acceleration(decelerate)
        else:
            self.bw.speed = const.FORWARD_SPEED

        if backwards:
            self.bw.backward()
        else:
            self.bw.forward()
        time.sleep(sec)
        
        
    def avoidance_turn(self, angle):
        self.is_acc = False
        self.fw.turn(angle)
        self.bw.forward()
        self.bw.speed = const.AVOIDANCE_TURN_SPEED
        time.sleep(1)


    def stop(self, sec=0):
        self.is_acc = False
        self.bw.stop()
        self.fw.turn_straight()
        time.sleep(sec)


    def obstacle_avoidance(self):
        temps_droite = const.AVOIDANCE_TURN_TIME
        temps_gauche = const.AVOIDANCE_TURN_TIME * const.LEFT_WHEEL_FACTOR
        virage_droite = const.AVOIDANCE_RIGHT_ANGLE
        virage_gauche = const.AVOIDANCE_LEFT_ANGLE
        
        # gauche
        self.avoidance_turn(virage_gauche)
        time.sleep(temps_gauche)

        # tout droit
        self.avoidance_turn(90)
        time.sleep(const.AVOIDANCE_STRAIGHT_TIME_EX)

        # droite
        self.avoidance_turn(virage_droite)
        time.sleep(temps_droite)
        
        # tout droit
        self.avoidance_turn(90)
        time.sleep(const.AVOIDANCE_STRAIGHT_TIME_MID)
        
        # droite
        self.avoidance_turn(virage_droite)
        time.sleep(const.AVOIDANCE_STRAIGHT_TIME_EX)

            
    def accelerate(self):
        self.speed = self.speed + (const.ACC_RATE * const.ACC_TIME) * (-1 if self.is_dec else 1)

    
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
        percentage_speed = (self.speed/const.MAX_SPEED_RATE)*100.0
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
            can_acc = self.speed < const.MAX_SPEED_RATE*(const.FORWARD_SPEED/100.0)
        else:
            self.is_acc = False
        
        if can_acc:
            if np.abs(time.time() - self.start_time_acc) >= const.ACC_TIME:
                self.start_time_acc = time.time()
                return True
        
        return False     
        
        
    def run(self):
        count = 0
        self.fw.turn_straight()

        while True:
            if self.ua.detect_obstacle():
                print("OBSTACLE DETECTED")
                self.stop(const.AVOIDANCE_STOP_TIME)
                self.move_by_dist(const.AVOIDANCE_BACKWARDS_DIST, backwards=True)
                self.stop(1)
                self.obstacle_avoidance()
                self.ua.clear_measures()
                self.end_obs_avoidance = True
                
            # Read line sensor
            turning_angle, turn_direction, is_off_track = self.lf.follow_line(const.LINE_STEP)

            if self.end_obs_avoidance and is_off_track:
                # Just finished avoidance but hasnt find the line
                self.bw.speed = const.FORWARD_SPEED
            else:
                # Found the line or the end of the trajectory

                if turning_angle == const.END_LINE:
                    print('Trajectory end')
                    self.stop()
                    break
                
                self.fw.turn(turning_angle)
                self.end_obs_avoidance = False
                
                if turn_direction == const.LEFT: # if LEFT, left wheel slower
                    self.bw.right_speed(const.DRIFT_MASTER_WHEEL_SPEED)
                    self.bw.left_speed(const.DRIFT_PIVOT_WHEEL_SPEED)
                    self.turn_count += 1
                    
                    if self.turn_count >= 7:
                        self.bw.left_forward(False)
                    
                elif turn_direction == const.RIGHT: # if RIGHT, right wheel slower
                    self.bw.right_speed(const.DRIFT_PIVOT_WHEEL_SPEED)
                    self.bw.left_speed(const.DRIFT_MASTER_WHEEL_SPEED)
                    self.turn_count += 1
                    
                    if self.turn_count >= const.DRIFT_START_COUNT:
                        self.bw.right_forward(False)
                    
                else:
                    self.turn_count = 0
                    self.move(backwards=False, acc=True,decelerate=False)
            
            if self.check_acceleration():
                self.accelerate()
                self.apply_speed()

            time.sleep(const.SENSOR_READ_DELAY)
            count += 1


def main():
    picar.setup()
    time.sleep(const.INITIAL_WAIT)
    c = Car()
    try:
        c.run()
    except KeyboardInterrupt:
        c.stop()


if __name__ == '__main__':
    main()