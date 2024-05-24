import minimalmodbus
import time
import sys
import tty
import termios

class encoderMotor():
    def __init__(self, slave_address):
        port = '/dev/ttyAMA0' # Serio port for UART

        self.instrument = minimalmodbus.Instrument(port, slave_address, minimalmodbus.MODE_ASCII)

        self.instrument.serial.baudrate = 9600
        self.instrument.serial.timeout = 0.1
        
    def enable_digital_mode(self,direction):
        if direction == 'forward':
            self.reg_status = self.instrument.write_register(2, 257, number_of_decimals=0, functioncode=6, signed=False) # Enable motor in CW
        if direction == 'reverse':
            self.reg_status = self.instrument.write_register(2, 265, number_of_decimals=0, functioncode=6, signed=False) # Enable motor in CCW
        if self.reg_status == 1:
            return True
        else:
            return False
        
    def disable_digital_mode(self,direction):
        if direction == 'forward':
            self.reg_status = self.instrument.write_register(2, 256, number_of_decimals=0, functioncode=6, signed=False) # Disable motor in CW
        if direction == 'reverse':
            self.reg_status = self.instrument.write_register(2, 264, number_of_decimals=0, functioncode=6, signed=False) # Disable motor in CCW
        if self.reg_status == 1:
            return True
        else:
            return False

    def forward(self,wheel_speed):   # Range : 0-65535,Default - 2048, Only int allowed
        motor_speed = 60* wheel_speed
        if motor_speed >18000:
            motor_speed = 18000  # Limit the speed to 18000 rpm which is RMCS-3015 motor base speed 
            
        self.reg_status = self.instrument.write_register(14, motor_speed, number_of_decimals=0, functioncode=6, signed=False) # speed
        self.enable_digital_mode('forward')
        if self.reg_status == 1:
            return True
        else:
            return False
        
    def reverse(self,wheel_speed):   # Range : 0-65535,Default - 2048, Only int allowed
        motor_speed = 60* wheel_speed
        if motor_speed >6000:
            motor_speed = 6000  # For reverse direction limit the speed to 1/3rd of max rpm of RMCS-3015 motor base speed = 6000 rpm
            
        self.reg_status = self.instrument.write_register(14, motor_speed, number_of_decimals=0, functioncode=6, signed=False) # speed
        self.enable_digital_mode('reverse')
        if self.reg_status == 1:
            return True
        else:
            return False
    
    def get_speed(self): 
        motor_speed = self.instrument.read_register(24, number_of_decimals=0, functioncode=3, signed=False)
        #print('Speed value :',speed)
        
        if motor_speed>32765:
            wheel_speed = (motor_speed-65535)/60 # Negative values for reverse speed
        else:
            wheel_speed = motor_speed/60
        
        return wheel_speed
        
    def get_acceleration(self): 
        acceleration = self.instrument.read_register(12, number_of_decimals=0, functioncode=3, signed=False)
        return acceleration
    
    def set_acceleration(self,acceleration):
        self.reg_status = self.instrument.write_register(12, acceleration, number_of_decimals=0, functioncode=6, signed=False) 
        if self.reg_status == 1:
            return True
        else:
            return False
    
    def get_lines_per_rot(self): 
        lpr = self.instrument.read_register(10, number_of_decimals=0, functioncode=3, signed=False)
        return lpr
    
    def set_lines_per_rot(self,lpr):
        self.reg_status = self.instrument.write_register(10, lpr, number_of_decimals=0, functioncode=6, signed=False) # default lines per rotation: 13
        if self.reg_status == 1:
            return True
        else:
            return False
        
    def brake_motor(self,direction): 
        if direction == 'forward':
            self.reg_status = self.instrument.write_register(2, 260, number_of_decimals=0, functioncode=6, signed=False) # Brake motor in CW
        if direction == 'reverse':
            self.reg_status = self.instrument.write_register(2, 268, number_of_decimals=0, functioncode=6, signed=False) # Brake motor in CCW
        if self.reg_status == 1:
            return True
        else:
            return False
        
    def enable_position_mode(self):
        self.reg_status = self.instrument.write_register(2, 513, number_of_decimals=0, functioncode=6, signed=False) # enable motor in position control mode
        if self.reg_status == 1:
            return True
        else:
            return False
        
    def disable_position_mode(self):
        self.reg_status = self.instrument.write_register(2, 512, number_of_decimals=0, functioncode=6, signed=False) # enable motor in position control mode
        if self.reg_status == 1:
            return True
        else:
            return False
        
    def absolute_position_count(self,count): #count = 3120 for each rotation
        if (count > 0 and count < 2147483647):
            data = count                         #(1 - 2147483646)
        if (count < 0 and count >= -2147483648):
            data = 4294967295 - (count * -1)     #(4294967294 - 2147483648)
        
        LSB_data = data & 0xFFFF # Extract lower two bytes
        MSB_data = data >> 16    # Extract upper two bytes
        print("data :", data)
        print("LSB_data :", LSB_data, ";", "MSB_data :", MSB_data)
        
        self.instrument.write_register(16, int(LSB_data), number_of_decimals=0, functioncode=6, signed=False) #LSB
        self.instrument.write_register(18, int(MSB_data), number_of_decimals=0, functioncode=6, signed=False) #MSB 
        self.enable_position_mode()
        
    def get_position(self): # in cm
        counts_per_rotation = 3120
        distance_cm_per_rot = 22
        pos_LSB = self.instrument.read_register(20, number_of_decimals=0, functioncode=3, signed=False)
        pos_MSB = self.instrument.read_register(22, number_of_decimals=0, functioncode=3, signed=False)
        pos = (pos_MSB<<16) + pos_LSB
        if pos>2147483647:
            pos = pos-4294967295
        return (pos/counts_per_rotation)*distance_cm_per_rot
     
    def set_acceleration(self,acceleration): # 0-65535, Default - 20000
        self.reg_status = self.instrument.write_register(12, acceleration, number_of_decimals=0, functioncode=6, signed=False) 
        if self.reg_status == 1:
            return True
        else:
            return False
        
    def reset_encoder(self): 
        self.reg_status = self.instrument.write_register(2, 2048, number_of_decimals=0, functioncode=6, signed=False) 
        if self.reg_status == 1:
            return True
        else:
            return False
    
    def stop_motor(self): # Stops motion but motor maintains position
        self.reg_status = self.instrument.write_register(2, 1793, number_of_decimals=0, functioncode=6, signed=False) 
        if self.reg_status == 1:
            return True
        else:
            return False
        
    def shutdown_motor(self): #Stops motion and power supply to motor is cut off
        self.reg_status = self.instrument.write_register(2, 1792, number_of_decimals=0, functioncode=6, signed=False) 
        if self.reg_status == 1:
            return True
        else:
            return False
        

# if __name__ == '__main__':
    
#     try:
#         left_motor = encoderMotor(6)
#         right_motor = encoderMotor(7)
#         left_motor.set_lines_per_rot(13)
#         right_motor.set_lines_per_rot(13)
#         print("Left Motor acceleration = ", left_motor.get_acceleration())
#         print("Right Motor acceleration = ", right_motor.get_acceleration())
#         print("Left Motor lpr = ", left_motor.get_lines_per_rot())
#         print("Right Motor lpr = ", right_motor.get_lines_per_rot())
#         
#         left_motor.reset_encoder()
#         right_motor.reset_encoder()
#         print("Left Motor initial position = ", left_motor.get_position())
#         print("Right Motor initial position = ", right_motor.get_position())
#         
#         
#         while True:   
#             #Move forward
#             speed = 5
#             left_motor.reverse(speed)
#             right_motor.forward(speed)
#             
#             #Move reverse
#             left_motor.forward(200)
#             right_motor.reverse(200)
#             
#             #time.sleep(5)
#             print("Left Motor speed = ", left_motor.get_speed())
#             print("Right Motor speed = ", right_motor.get_speed())
#             print("Left Motor position = ", left_motor.get_position())
#             print("Right Motor position = ", right_motor.get_position())
#         
#             left_motor.brake_motor('forward')
#             right_motor.brake_motor('forward')
#             time.sleep(0.1)
          
        
#     except KeyboardInterrupt:
#         #when control c is pressed it will clean up the GPIO from input and output
#         print("Exitted") 

