import numpy as np
from math import *
import time

stuck_start_time = time.time()
donut_start_time = time.time()

#The requirement for a passing submission is to map at least 40% of the environment at 60% fidelity
#  and locate at least one of the rock samples. Each time you launch the simulator in autonomous
# mode there will be 6 rock samples

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function

def find_nearest(array,value):

    print("looking for closest item to", value)
    #print(array * 180/np.pi)
    idx = (np.abs(array-value)).argmin()
    print("index,value", idx, array[idx])
    # print(np.abs(array-value))
    return array[idx]

def take_evasive_action(Rover):
    global stuck_start_time

    print("***TAKING EVASIVE ACTION****")
    #print(stuck_start_time, time.time() - stuck_start_time)
    # # if stuck_start_time==None:
    # #     stuck_start_time=time.time()
    #print("stuck more than 0.5 -- Initiating Unstuck-ing Proceudre")
    Rover.throttle = 0
    # Release the brake to allow turning
    Rover.brake = 0
    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
    Rover.steer = -15
    # stuck_start_time = time.time()
    return Rover


def decision_step(Rover):

    global stuck_start_time
    global donut_start_time



    #done?
    if Rover.mode == "mission complete":
        quit()


    #send pickup?
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        print("SENDING PICKUP")
        Rover.send_pickup = True
        return Rover

    #STOP is we're near a sample
    if Rover.near_sample:
        print("near sample, trying to stop")
        Rover.mode = 'stop'
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        # Rover.send_pickup = True
        return Rover

    #are we in the middle of a donut condition and trying to break out of it?
    if Rover.mode == "donut_breakout":
        print("handling donut breakout procedure")
        if time.time() - donut_start_time > 15:
            Rover.mode = "forward"
            donut_start_time=time.time()
        else:
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = -15
        return Rover

    #detect if our vehicles is doing donuts
    if Rover.steer==15 or Rover.steer==-15:
        #how long have we been doing donuts?
        if time.time() - donut_start_time > 10:  #more than 10 seconds?...try to break out
            print("Detected that you're stuck in a donut...", time.time() - donut_start_time)
            #Rover.steer= Rover.steer*-1
            #donut_start_time = time.time()
            Rover.mode = "donut_breakout"
            return Rover
    else:
        donut_start_time=time.time()

    #This shouldn't happen, but here for logging purposes. Did we overshoot the pickup?
    if Rover.picking_up:
        if not Rover.near_sample:
            print("Overshot the sample!  Reacquiring..")
            # Rover.brake = 0
            # Rover.throttle = -2
            Rover.mode='stop'
            Rover.picking_up=0
            Rover.send_pickup = False
            Rover.steer=-15
        return Rover


    #Steer to direction of the nugget
    if Rover.nugget_mean_angle!=None and not isnan(Rover.nugget_mean_angle):
        Rover.steer = degrees(Rover.nugget_mean_angle)
        Rover.throttle = Rover.throttle_set
        Rover.brake = 0
        return Rover

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        
        # Check for Rover.mode status
        if Rover.mode == 'forward':

            # #check if STUCK under a rock but can see navigable terrain
            if Rover.vel < 0.1 and Rover.throttle!=0:
                if time.time() - stuck_start_time > 0.5:
                    Rover = take_evasive_action(Rover)
                    return Rover

            #reset time spent stuck
            else:
                stuck_start_time = time.time()


            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                try:
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                except:
                    print("mean of nav angles caught an exception")
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    try:
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    except:
                        print("mean of nav angles caught an exception")
                    Rover.mode = 'forward'



    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = 0 #Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    #save start pos
    if Rover.start_pos==None:
        Rover.start_pos = ( Rover.pos[0],Rover.pos[1])
        print("Saving start position as:", Rover.start_pos)

    #calculate distance to home and Quit if we're there and have all 6 samples
    Rover.distance_home = sqrt((Rover.pos[1] - Rover.start_pos[1])**2 +(Rover.pos[0] - Rover.start_pos[0])**2)
    if Rover.samples_found == 6:
        if Rover.distance_home < 5:
            print("Mission Complete, 6 samples collected, returned to <5 meters from start point")
            Rover.mode = "mission complete"
            Rover.throttle = 0
            Rover.steering = 0
            Rover.brake = Rover.brake_set
            return Rover
    return Rover

