import RPi.GPIO as GPIO
from time import sleep
from uss_driver import uss_driver

GPIO.setmode(GPIO.BCM)

class uss:
    def __init__(self):
              
        self.uss_front = uss_driver(trig=18, echo=23)
        self.uss_rear  = uss_driver(trig=24, echo=25)
    #     self.uss_left  = uss_driver(trig=8, echo=7)
    #     self.uss_right = uss_driver(trig=12, echo=16)


    def getDistance(self,uss_location):
        
        if uss_location == 'front':
            return self.uss_front.getDistance()
        elif uss_location == 'rear':
            return self.uss_rear.getDistance()
        elif uss_location == 'left':
            return self.uss_left.getDistance()
        elif uss_location == 'right':
            return self.uss_right.getDistance()
        else:
            print('Invalid location of USS. Only front, rear, left or right allowed')
            return 0
    
    def getSurroundDistance(self): # returns all four USS sensor distances in order - front,rear,left and right
        
      return [self.uss_front.getDistance(), self.uss_rear.getDistance() , self.uss_left.getDistance(), self.uss_right.getDistance()]

    def __del__(self):
        # print("USS is deleted")
        del self.uss_front
        del self.uss_rear
        # del self.uss_left
        # del self.uss_right


if __name__ == "__main__":
    uss = uss()
    
    try:
        while True:
            # printing obstacle distance every 1 second
            print("inside")
            print("Front USS Distance = ", uss.getDistance('front'))
            print("Rear USS Distance = ", uss.getDistance('rear'))
            sleep(0.5)
            
    except KeyboardInterrupt:
        # when control c is pressed it will clean up the GPIO from input and output
        print("Exitted")
        del uss
