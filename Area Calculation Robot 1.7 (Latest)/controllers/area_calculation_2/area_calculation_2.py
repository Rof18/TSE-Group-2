"""area_calculation controller."""
from controller import Robot
from controller import DistanceSensor
from controller import PositionSensor
from controller import GPS
import math
import sys
       

#function to set motor velocity to rotate left in place        
def motor_rotate_left():
    frontleft_motor.setVelocity(-max_speed)
    backleft_motor.setVelocity(-max_speed)
    
    frontright_motor.setVelocity(max_speed)
    backright_motor.setVelocity(max_speed) 
    
#function to set motor velocity to move forward
def motor_move_forward():
    frontleft_motor.setVelocity(max_speed)
    backleft_motor.setVelocity(max_speed)
    
    frontright_motor.setVelocity(max_speed)
    backright_motor.setVelocity(max_speed)    
    
#function to stop the motor set motor velocity to zero  
def motor_stop():
    frontleft_motor.setVelocity(0)
    backleft_motor.setVelocity(0)
    
    frontright_motor.setVelocity(0)
    backright_motor.setVelocity(0)  
    
 
#function to set suration of the rotation
def motor_rotate_left_in_degrees():
    motor_rotate_left()
    duration = 0.8
    start_time = robot.getTime()
    while robot.getTime() < start_time + duration:
        robot.step(timestep)
    motor_stop()
    
#function to read gps values   
def get_gps_values():
     coordinates = []
     gps_value = gps.getValues() 
     for i in range(len(gps_value)-1):
          coordinates.append(gps_value[i])
     sets.append(coordinates)
    
#funtcion to calulate areas of triangles from coordinates      
def calculate_area():
    determinant = [0,0,0,0]
    total_area = 0.0
    temp = 1
    for i in range(len(sets)-2):
       determinant[0] = sets[0][0] - sets[temp][0]
       determinant[1] = sets[0][0] - sets[temp + 1][0]
       determinant[2] = sets[0][1] - sets[temp][1]
       determinant[3] = sets[0][1] - sets[temp + 1][1]
       total_area = total_area + ((determinant[0] * determinant[3])-(determinant[1] * determinant[2])) / 2
       temp = temp + 1
    print(sets)
    print("Total area calculated: ","{0:0.5f}m\u00b2".format(total_area ))
       
       
# Main loop:
# - perform simulation steps until Webots is stopping the controller      
if __name__ == "__main__":
# creates the Robot instance.
    robot = Robot()
    timestep = 64
    max_speed = 6.28
    
    #initiate robots motors
    frontright_motor = robot.getDevice('fr_motor_1')
    frontleft_motor = robot.getDevice('fl_motor_2')
    backright_motor = robot.getDevice('br_motor_3')
    backleft_motor = robot.getDevice('bl_motor_4')
    
    frontright_motor.setPosition(float('inf'))
    frontright_motor.setVelocity(0)
    frontleft_motor.setPosition(float('inf'))
    frontleft_motor.setVelocity(0)
    
    backright_motor.setPosition(float('inf'))
    backright_motor.setVelocity(0)
    backleft_motor.setPosition(float('inf'))
    backleft_motor.setVelocity(0)
    
    #initiate and enable robots sensors  
    dsensor1 = robot.getDevice('ds_0')
    dsensor1.enable(timestep)
    dsensor2 = robot.getDevice('ds_1')
    dsensor2.enable(timestep)
    
    
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    finished = 0;  
    sets = []
    
    while robot.step(timestep) != -1:
       ##print_sensor_values()
        
       if dsensor1.getValue() < 650 or dsensor2.getValue() < 650:
           motor_rotate_left_in_degrees()
           get_gps_values()
           finished = finished + 1
       elif finished == 10: 
           calculate_area()
           sys.exit(0)   
       else:
           motor_move_forward()

   
