"""controllerOne controller."""

from controller import Robot

# Proportional-Derivative Control 
def runRobot(robot):

    # Timestep
    t = int(robot.getBasicTimeStep())
    
    # Collect our Robots Motor
    motor = robot.getDevice('motor')
    # 'inf' and '0' mean out motor's velocity is subject to change within our main loop
    motor.setPosition(float('inf'))
    motor.setVelocity(0)
    
    # Gyroscope to measure the robot's angle
    gyro = robot.getDevice('gyro')
    gyro.enable(t)
    
    # Proportional Gain, we use an approximate as a constant
    Kc = 5
        #When Kc is too high, the Robot accelerates sparratically
    
    # Derivative time constant, another approximation
    Td = .6
        #When Td is too high, the Robot over-corrects

    # The current error in the robots tilt, initialized at 0
    e = 0.0
    
    # The current velocity output by the motor, initialized at 0
    c = 0
    
    while robot.step(t) != -1:
    
        # Read the gyroscope to get tilt at time t, this will be our new error
        gyroRead = gyro.getValues()
      
        # gyroRead is a list shaped like [x, y, z]
            # Our robot tilts only on the y axis, so we only need the y angle reads
        e2 = gyroRead[1]
        
        # Function for Robots correction
            # c = Kc(e + Td *(delta_e/delta_t)) + c
        
        c += Kc * (e + Td*(e2-e)/t)

        # Set the new Velocity
        motor.setVelocity(c)
        
        # The last error read is now the current error read for the next loop
        e = e2
        
        pass
        
# Create the Robot instance.
if __name__ == "__main__":

    robot = Robot()
    runRobot(robot)
    
# Areas of Improvement:
    # Genetic Algorithm to replace Approximated constants
    
    # Adjustments for a second Axis of tilt
