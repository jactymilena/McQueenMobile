# from SunFounder_Line_Follower import Line_Follower
from SunFounder_Ultrasonic_Avoidance import Ultrasonic_Avoidance
from picar import front_wheels
from picar import back_wheels
import time
import picar
import constants as const


class Car:
    def __init__(self):
        self.ua = Ultrasonic_Avoidance.Ultrasonic_Avoidance(20)
        self.fw = front_wheels.Front_Wheels(db='config')
        self.bw = back_wheels.Back_Wheels(db='config')
        self.fw.turning_max = 45


    def move(self, sec=0, backwards=False):
        self.fw.turn_straight()
        if backwards:
            self.bw.backward()
            self.bw.speed = const.BACKWARD_SPEED
        else:
            self.bw.forward()
            self.bw.speed = const.FORWARD_SPEED
        time.sleep(sec)
        
    
    def turn_right(self, sec=2.3):
        self.fw.turn_right()
        self.bw.forward()
        self.bw.speed = const.FORWARD_SPEED
        time.sleep(sec)


    def turn_left(self, sec=2.3): 
        self.fw.turn_left()
        self.bw.forward()
        self.bw.speed = const.FORWARD_SPEED
        time.sleep(sec)


    def turn(self, angle):
        self.fw.turn(angle)
        self.bw.forward()
        self.bw.speed = const.FORWARD_SPEED
        time.sleep(1)


    def stop(self, sec=0):
        self.bw.stop()
        self.fw.turn_straight()
        time.sleep(sec)


    def obstacle_avoidance(self):
        self.turn_right()
        self.move(2)
        self.turn_left()
        self.move(2)
        self.turn_left()
        self.move(2)
        self.turn_right()


    def run(self):
        count = 0

        while True:
            if self.ua.detect_obstacle():
                print("OBSTACLE DETECTED")
                self.move(1, backwards=True)
                self.stop(2)
                self.obstacle_avoidance()
            else:
                self.move()

            count += 1

            if count > 120:
                self.stop()
                # Trajectory end (TODO add line follower check for end)
                break


    def test(self):
        self.obstacle_avoidance()
        # self.stop()

        # self.fw.turn_straight()
        # time.sleep(1)
        # self.fw.turn_right()
        # self.bw.forward()
        # self.bw.speed = const.FORWARD_SPEED
        # time.sleep(2.5)
        # self.fw.turn_straight()
        # time.sleep(1)
        self.stop()


def main():
    picar.setup()
    time.sleep(5)
    c = Car()
    try:
        c.run()
    except KeyboardInterrupt:
        c.stop()


if __name__ == '__main__':
    main()