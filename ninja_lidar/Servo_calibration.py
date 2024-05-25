import RPi.GPIO as GPIO
from time import sleep
import readchar
from Servo import Servo
from FileManagement import fileDB 

steer = Servo()

manual = '''
----------------------------- Calibration ----------------------------------
[1]: steering servo                                [W]: increase servo angle
[2]: pan servo                                     [S]: decrease servo angle
[3]:tilt servo


[SPACE] confirm calibration                                  [Ctrl+C]: quit
'''

servo_names = ['steering', 'pan', 'tilt']
servos_offset = [0,0,0]
servo_num = 0 # A pointer for the list


def show_info():
    #clear display
    print("\033[H\033[J", end='')
    print(manual)
    print('[%s]'%(servo_names[servo_num]))
    print('offset: %s'%(servos_offset))
    
    
def set_servos_offset(servo_num, value):
    if servo_num == 0:
        steer.steer_servo_conf_val = value


def calibration():
    global servo_names, servo_num, servos_offset
    
    #resetting to previously servo conf value
    steer.set_SteeringAngle(0)
    
    step_size = 3 #step size of servo
    
    show_info()
    
    while True:
        key = readchar.readkey()
        key = key.lower()
        
        
        if key in ('123'):
            servo_num = int(key)-1
            show_info()
            
        elif key == 'w':
            servos_offset[servo_num] += step_size
#             if servos_offset[servo_num] > 30: #max limit
#                 servos_offset[servo_num] = 30
            show_info()

            set_servos_offset(servo_num, servos_offset[servo_num]) #save values in the class variable
            if(servo_num == 0):
                steer.set_SteeringAngle(servos_offset[servo_num])
                
        elif key == 's':
            servos_offset[servo_num] -= step_size
#             if servos_offset[servo_num] < -30: #max limit
#                 servos_offset[servo_num] = -30
            show_info()
            
            set_servos_offset(servo_num, servos_offset[servo_num]) #save values in the class variable
            if(servo_num == 0):
                steer.set_SteeringAngle(servos_offset[servo_num]) # set the steering angle
                
        elif key == readchar.key.SPACE:
            print('Confirm save? (y/n)')
            while True:
                key = readchar.readkey()
                key=key.lower()
                if key == 'n':
                    show_info()
                    break
                elif key == 'y':
                    steer.set_servo_conf(value = servos_offset[0]) #save info in the file
                    sleep(0.2)
                    servos_offset = [steer.steer_servo_conf_val, 0, 0] #show info of servo
                    show_info()
                    print('The calibration value has been saved')
                    break
                sleep(0.01)
            
                
    
if __name__ == "__main__":
    try:
        calibration()
    except KeyboardInterrupt:
        print('quit')
    finally:
        GPIO.cleanup()
#         steer.set_SteeringAngle(0)
        del steer
        sleep(0.1)
    