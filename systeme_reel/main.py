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


    def move(self, backwards=False):
        self.fw.turn_straight()
        if backwards:
            self.bw.backward()
            self.bw.speed = const.BACKWARD_SPEED
        else:
            self.bw.forward()
            self.bw.speed = const.FORWARD_SPEED
        time.sleep(1)
        

    def turn(self, angle):
        self.fw.turn(angle)
        self.bw.forward()
        self.bw.speed = const.FORWARD_SPEED
        time.sleep(1)


    def run(self):
        count = 0

        while True:
            avoid_flag = self.ua.avoid_obstacle()
            

            if avoid_flag == const.BACKWARDS_FLAG:
                # Obstacle detected too close, move backwards
                print("BACKWARDS")
                self.move(True)
                self.stop()
                break
            elif avoid_flag == const.TURN_FLAG:
                # Obstacle detected, turn right   
                print("TURN")        
                self.turn(90)
            else:
                print("MOVE")
                self.move()
                # self.fw.turn_straight()

            count += 1

            if count > 120:
                self.stop()
                # Trajectory end (TODO add line follower check for end)
                break


    def stop(self):
        self.bw.stop()
        self.fw.turn_straight()


def main():
    picar.setup()
    # stop()
    c = Car()
    # c.run()
    try:
        c.run()
    except KeyboardInterrupt:
        c.stop()



if __name__ == '__main__':
    main()