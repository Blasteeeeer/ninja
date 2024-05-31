from picarx import Picarx
import time

POWER = 1
SafeDistance = 25   # > 30 safe
DangerDistance = 3 # > 20 && < 30 turn around, 
                    # < 20 backward

def main():
    try:
        px = Picarx()
        # px = Picarx(ultrasonic_pins=['D2','D3']) # tring, echo
       
#         px.set_dir_servo_angle(0)
        #while True:
        distance = round(px.ultrasonic.read(), 2)
        print("distance: ",distance)
#         if distance >= SafeDistance:
#             Obstacle_status = 'Straight'
# #             px.set_dir_servo_angle(0)
# #             px.forward(POWER)
        if distance <= DangerDistance:
#             Obstacle_status = 'Right'
#             px.set_dir_servo_angle(25)
            px.forward(0)
            time.sleep(0.5)
#         else:
#             Obstacle_status = 'Left'
# #             px.set_dir_servo_angle(-25)
# #             px.backward(POWER)
#             time.sleep(0.1)

    finally:
        #px.forward(0)
#         Obstacle_status
        pass


