import numpy as np
# Rover.mode
# Rover.throttle
# Rover.brake
# Rover.steer
# Rover.nav_angles
# Rover.stop_forward
# Rover.go_forward


def getLeftDistance(nav_angles, nav_distance):
    left_indeces = nav_angles > np.deg2rad(33)
    if np.any(left_indeces):
        return np.mean(nav_distance[left_indeces]) / 10
    else:
        return 0


def getRightDistance(nav_angles, nav_distance):
    right_indeces = nav_angles < np.deg2rad(-33)
    if np.any(right_indeces):
        return np.mean(nav_distance[right_indeces]) / 10
    else:
        return 0


def decision_step(Rover):
    # There are three main modes of operation
    # Go to Edge (go)
    # Stay With Edge (stay)
    # Stop and Turn (stop)
    if Rover.mode == "go-rotate":
        print("go-rotate")
        # make the rover turn
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = -15
        # stop turning when the rover is close to a wall
        if len(Rover.nav_angles) < 1800:
            Rover.steer = 0
            Rover.mode = "go"
    elif Rover.mode == "go":
        print("go")
        # go to the wall
        Rover.steer = 0
        Rover.throttle = Rover.throttle_set
        # if the rover is close to a wall go to sop mode
        if len(Rover.nav_angles) < Rover.go_forward:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.mode = "stop"
    elif Rover.mode == "stay":
        print("Stay With Edge")
        if len(Rover.nav_dists) > 0 and len(Rover.nav_angles) > 0:
            Rover.brake = 0
            Rover.throttle = Rover.throttle_set
            left_distance = getLeftDistance(Rover.nav_angles, Rover.nav_dists)
            right_distance = getRightDistance(Rover.nav_angles, Rover.nav_dists)
            target_distance = np.min([1.5, (left_distance + right_distance) / 2.0])
            steerDif = left_distance - target_distance
            if steerDif > 0:
                Rover.steer = np.clip(10*steerDif, -15, 10)
            else:
                Rover.steer = np.clip(30*steerDif, -15, 10)
            if len(Rover.nav_angles) < Rover.go_forward:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.mode = "stop"
        if Rover.rock_found:
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            Rover.steer = 0
            Rover.prev_mode = Rover.mode
            Rover.mode = "rock"
    elif Rover.mode == "rock":
        print("rock")
        Rover.brake = 0
        if len(Rover.rock_angles) > 0:
            if abs(Rover.vel) < 0.09:
                Rover.throttle = 0
            Rover.steer = np.clip(np.mean(Rover.rock_angles)*180/np.pi, -15, 15)
            Rover.throttle = Rover.throttle_set
        else:
            Rover.rock_found = False
            Rover.mode = Rover.prev_mode
        if Rover.near_sample:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
    elif Rover.mode == "stop":
        print("stop")
        Rover.steer = -15
        Rover.brake = 0
        if len(Rover.nav_angles) > Rover.go_forward:
            Rover.mode = "stay"
            Rover.steer = 0
    elif Rover.mode == "stuck":
        print("stuck")
        time_stuck = Rover.total_time - Rover.first_stuck
        if time_stuck % 4.0 < 2.0:
            Rover.brake = 0
            Rover.steer = -15
            Rover.throttle = 0
        else:
            Rover.brake = 0
            Rover.steer = 0
            Rover.throttle = Rover.throttle_set * 2
        if Rover.vel > 0.2:
            Rover.mode = "stay"
            Rover.first_stuck = None

    if Rover.throttle > 0 and abs(Rover.vel) < 0.09 and Rover.mode != 'stuck' and Rover.mode != 'rock':
        if Rover.first_stuck == None:
            Rover.first_stuck = Rover.total_time
        elif (Rover.total_time - Rover.first_stuck) > 9.0:
            Rover.mode = "stuck"

    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.mode = Rover.prev_mode

    return Rover
