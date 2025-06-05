from time import sleep
from pwm_control import Servo
from stepper_control import DRV8825

# Initialize the stepper driver with GPIO pins 16 and 17, and 2000 steps per revolution.
stepper_driver = DRV8825(16, 17, 2000)

# Create a Servo instance to control the servo motor on GPIO 15, then activate it.
servo = Servo(15)
servo.start()

# The two pipes are connected, while the syringe's pipe is closed. 
# This option is not used in the program.
# servo.set_angle(180)
# sleep(t)  # Pause

# Number of steps for the stepper motor
# Moves forward in full steps, then backward.
# Full steps are the default mode but can be changed.
steps = 100

t = 2  # 2-second pause

while True:
    
    sleep(t)  # Pause
        
    # When the servo motor is at 90 degrees, the right pipe is connected to the syringe.
    # The middle pipe is closed, meaning the colored water is drawn from the ground.
    servo.set_angle(90)
    sleep(t)  # Pause

    i = 0
        
    # The stepper motor moves backward, drawing up the colored water with the syringe. 
    while i > -10:      
    
        stepper_driver.step(steps, -1)
        i -= 1
        print("Moving backward:", i)
        
        sleep(t)       
            
    # When the servo motor is at 0 degrees, the middle pipe is connected to the syringe.
    # The right pipe is closed, meaning the colored water is pushed into the infusion bag.
    servo.set_angle(0)
    sleep(t)  # Pause       
     
    j = 0

    # The stepper motor moves forward, injecting the colored water into the infusion bag.
    # It performs one step less because the system pushes more than it pulls. 
    while j < 10:
    
        stepper_driver.step(steps, 1)
        j += 1
        print("Moving forward:", j)
        sleep(t)
           
    sleep(t)