import RPi.GPIO as GPIO
import time

class uss_driver:
    trig, echo = 0, 0
    def __init__(self, trig, echo):
        # set the numbering mode for the GPIO pins
        GPIO.setmode(GPIO.BCM)
        
        self.trig, self.echo = trig, echo
        # set the trigger and echo as output and input respectively
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        
    # it return -1 for timeout error
    # or it will return the distance
    def getDistance(self):
        # set trigger to low
        GPIO.output(self.trig, 0)
        time.sleep(0.01)
        
        # send high pulse to trigger for 10us
        GPIO.output(self.trig, 1)
        time.sleep(0.00001)
        GPIO.output(self.trig, 0)
        
        pulse_start = 0
        pulse_end = 0
        # wait until the echo pin is high and then start the timer. Which means echo is transmitted
        timeout = time.time() + 0.3 # for 1 second timout
        while GPIO.input(self.echo) == 0 and time.time() < timeout:
            pulse_start = time.time()

        if time.time() >= timeout:
            print("1st Timeout error")
            return -1
        
        # wait until the echo pin is low and stop the timer. Which means echo is recieved
        timeout = time.time() + 0.3 # for 1 second timout
        while GPIO.input(self.echo) == 1 and time.time() < timeout: 
            pulse_end = time.time()

        if time.time() >= timeout:
            print("2nd Timeout error")
            return -1
            
        # finding the duration from echo transmit to echo recieve
        pulse_duration = pulse_end - pulse_start
        
        # calculating the distance. distance = speed * time
        # divide by 2 since the echo has travel twice the distance from the object
        distance = pulse_duration * 34300/2
        
        return distance
    
    def __del__(self):
        # print("USS object deleted")
        GPIO.cleanup([self.trig, self.echo])
        
def setUssPins():
    # declaring object for USS
    uss_front = uss_driver(trig=18, echo=23)
    uss_back = uss_driver(trig=24, echo=25)
    uss_left = uss_driver(trig=8, echo=7)
    uss_right = uss_driver(trig=16, echo=20)

    

        


if __name__ == "__main__":
    print("Finding Distance from USS")
    
    # declaring object for USS
    uss_front = uss_driver(trig=18, echo=23)
    uss_back = uss_driver(trig=24, echo=25)
    uss_left = uss_driver(trig=8, echo=7)
    uss_right = uss_driver(trig=16, echo=20)

    try:
        while True:
            # printing obstacle distance every 1 second
            print("Front Distance = ", uss_front.getDistance())
            print("Back Distance = ", uss_back.getDistance())
            print("Left Distance = ", uss_left.getDistance())
            print("Right Distance = ", uss_right.getDistance())
            
            time.sleep(0.3)
        
        
    except KeyboardInterrupt:
        # when control c is pressed it will clean up the GPIO from input and output
        print("Exitted")
        del uss_front
        del uss_back
        del uss_left
        del uss_right

    