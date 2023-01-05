import numpy as np

# function that gets the average navigable distance
# to the rover's left
def getLeftDistance(nav_angles, nav_distance):
    left_indeces = nav_angles > np.deg2rad(33)
    if np.any(left_indeces):
        return np.mean(nav_distance[left_indeces]) / 10
    else:
        return 0

# function that gets the average navigable distance
# to the roever's right
def getRightDistance(nav_angles, nav_distance):
    right_indeces = nav_angles < np.deg2rad(-33)
    if np.any(right_indeces):
        return np.mean(nav_distance[right_indeces]) / 10
    else:
        return 0


def decision_step(Rover):
    # The Rover starts in the go-rotate state
    # it looks for a close enough wall/obstacle
    # it then transfers to the go mode
    if Rover.mode == "go-rotate":
        # for debugging purposes show current mode
        print("go-rotate")
        # make the rover turn right and look for a close wall
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = -15
        # stop turning when the rover is close enough to a wall
        if len(Rover.nav_angles) < 1800:
            Rover.steer = 0
            Rover.mode = "go"
    # The Rover is now pointing straight at a wall
    # it should now move until it almost hits the wall
    # once the rover brakes it transfers to the stop state
    elif Rover.mode == "go":
        # for debugging purposes show current mode
        print("go")
        # move straight to the wall
        Rover.steer = 0
        Rover.throttle = Rover.throttle_set
        # if the rover is close to a wall go to stop mode
        if len(Rover.nav_angles) < Rover.go_forward:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.mode = "stop"
    # This is the main mode of operation.
    # The Rover is now close to a wall on its left
    # It tries to stay a set distance away from the wall
    # It currently only uses proportional control to steer the Rover.
    elif Rover.mode == "stay":
        if Rover.picking_up:
            return Rover
        # for debugging purposes show current mode
        print("Stay With Edge")
        # if we have some vision data take a decision
        if len(Rover.nav_dists) > 0 and len(Rover.nav_angles) > 0:
            # keep the rover going steady
            Rover.brake = 0
            Rover.throttle = Rover.throttle_set
            # calculate the right and left distances of the rover to the walls
            left_distance = getLeftDistance(Rover.nav_angles, Rover.nav_dists)
            right_distance = getRightDistance(Rover.nav_angles, Rover.nav_dists)
            # the target distance should be 1.5m or the average between both
            # If the rover is in a narrow alley the average means that it tries to
            # stay equidistant to both edges
            target_distance = np.min([1.5, (left_distance + right_distance) / 2.0])
            steerDif = left_distance - target_distance
            # we want to choose different proportional control parameters for the
            # left and right steering. Basically, we don't mind the rover taking
            # its time with getting closer to the wall but we want to have it
            # get away from the wall quickly
            if steerDif > 0:
                # low Proportional control constant for left turning towards wall
                Rover.steer = np.clip(10*steerDif, -15, 10)
            else:
                # High P value for turning away from wall
                Rover.steer = np.clip(30*steerDif, -15, 10)
            # if we reach a dead end and can't move brake, go to the stop mode
            if len(Rover.nav_angles) < Rover.go_forward:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.mode = "stop"
        else:
            Rover.throttle = Rover.throttle_set
            Rover.brake = 0
        # regardless of our decisions move towards a rock when found
        # so go to rock mode
        print(np.mean(Rover.rock_dists))
        print(len(Rover.rock_dists))
        if Rover.rock_found and len(Rover.rock_dists) > 0 and np.mean(Rover.rock_dists) < 170 and np.mean(Rover.rock_angles) > 0:
            print(np.mean(Rover.rock_angles))
            Rover.steer = 0
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.prev_mode = Rover.mode
            Rover.mode = "rock"
    # Here we try to move towards a rock. If the rock is close stop moving
    # The Rover will then pickup the rock due to code written at the bottom of the file
    elif Rover.mode == "rock":
        print("Rock")
        Rover.throttle = Rover.throttle_set / 2
        Rover.brake = 0
        if (Rover.rock_angles.any()):
            Rover.rock_lost = True
            Rover.steer = np.clip(np.mean(Rover.rock_angles)*10,-15,15)
        elif not Rover.near_sample:
            Rover.first_rock = Rover.total_time
            if Rover.rock_lost:
                Rover.steer = 0
                Rover.throttle  = 0
                Rover.brake  = Rover.brake_set
                Rover.rock_lost = False
            else:
                Rover.steer = -7
                Rover.brake = 0
                Rover.throttle = 0
            if Rover.total_time - Rover.first_rock > 9.0:
                Rover.mode = "stay"
        if Rover.near_sample:
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            Rover.steer = 0
    # This mode is used when the rover encounters a dead end
    # or is about to hit a wall
    # the rover starts turning right to keep the wall to its left
    # once the rover finds enough navigable space to move go to stay mode
    elif Rover.mode == "stop":
        # for debugging purposes show current mode
        print("stop")
        # turn right to keep the wall to your left
        Rover.steer = -15
        Rover.brake = 0
        # if you have enough navigable space go to stay mode
        if len(Rover.nav_angles) > Rover.go_forward:
            Rover.mode = "stay"
            Rover.steer = 0
    # If the rover has not been moving for a while, the stuck mode is enabled
    # here the rover toggles between turning and throttling in an attempt
    # to get out of the situation
    # once the rover is moving control goes back to the stay mode
    elif Rover.mode == "stuck":
        # for debugging purposes show current mode
        print("stuck")
        # calculate the current time stuck
        time_stuck = Rover.total_time - Rover.first_stuck
        # for half the time steer right
        if time_stuck % 4.0 < 2.0:
            Rover.brake = 0
            Rover.steer = -15
            Rover.throttle = 0
        # for half the time throttle hard
        else:
            Rover.brake = 0
            Rover.steer = 0
            Rover.throttle = Rover.throttle_set * 2
        # once you move go back to stay mode
        if Rover.vel > 0.2:
            Rover.mode = "stay"
            Rover.first_stuck = None

    # Regardless of the current mode, if the rover is throlttling and not moving for 9 seconds go to 
    # stuck mode
    if Rover.throttle > 0 and abs(Rover.vel) < 0.09 and Rover.mode == "stay" and Rover.brake == 0:
        # once the rover is stuck start counting seconds
        if Rover.first_stuck is None:
            Rover.first_stuck = Rover.total_time
        # once 9 seconds are up go to stuck mode (the rover isn't doing so well)
        elif (Rover.total_time - Rover.first_stuck) > 9.0:
            Rover.prev_mode = Rover.mode
            Rover.mode = "stuck"

    # if the rover is near a rock and stopped pickup the rock
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.mode  = "stay"

    return Rover
