import readchar
import time

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..')) # including parent directory for importing
from drive import drive
from steer import Servo


manual = '''
----------------------------- Calibration ----------------------------------
[W]: increase speed                                [A]: move left
[S]: decrease speed                                [D]: move right

[B]: Brake                                 [Ctrl+C]: quit
'''

def show_info():
    #clear display
    print("\033[H\033[J", end='')
    print(manual)


def control(bot_drive, servo):
    show_info()
    speed = 0
    speed_step = 10
    steering_angle = 0
    steering_step_size = 3
    
    while True:
        key = readchar.readkey()
        key = key.lower()
        
        if key in 'wsadb':
            show_info()
        
        if key == 'w':
            print("Moved forward")
            speed = speed_step *2
            bot_drive.forward(speed)
            
        # elif key == 's':
        #     print("Moved backward")
        #     speed = speed_step/2
        #     bot_drive.reverse(speed)
            
        elif key == 'a':
            print("Moved left")
            steering_angle -= steering_step_size
            servo.set_SteeringAngle(steering_angle,Smoothnes = False)
            
        elif key == 'd':
            print("Moved right")
            steering_angle += steering_step_size
            servo.set_SteeringAngle(steering_angle,Smoothnes = False)
            
        elif key == 'b':
            print("Stoped")
            bot_drive.forward(0)
            
        
if __name__ == "__main__":
    
    try:
        bot_drive = drive(1) #single motor drive
        servo = Servo()
        control(bot_drive, servo)
    except KeyboardInterrupt:
        print('quit')
    finally:
        print('finally part')
        del bot_drive
        del servo
        pass
