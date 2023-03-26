from SunFounder_Line_Follower import Line_Follower
from SunFounder_Ultrasonic_Avoidance import Ultrasonic_Avoidance
from picar import front_wheels
from picar import back_wheels
import time
import picar
import constants as const


class Car:
    def __init__(self) -> None:
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

    def turn(self, angle):
        self.fw.turn()
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
            elif avoid_flag == const.TURN_FLAG:
                # Obstacle detected, turn right   
                print("TURN")        
                self.turn(rand_dir())
            else:
                print("MOVE")
                self.move()

            count += 1

            if count > 120:
                # Trajectory end (TODO add line follower check for end)
                break


def main():
    picar.setup()
    c = Car()
    c.run()


if __name__ == '__main__':
    main()