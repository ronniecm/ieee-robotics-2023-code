#!/usr/bin/env python3

#==========================================
# Title:  Robot Path Class
# Author: Azam Shoaib
# Date:   2 March 2023
#==========================================

from Robot import Robot as bot
from Ranging import Ranging as range
from RobotCommand import RobotCommand as command
import objectTrackingv5 as track
import getToLocation as location

'''
if we start from the white box we can do either 2 things:
    1. We can start off by dropping off the chips
        pro: easy points straight off the bat
        con: if we want to do the path of going right to left and up and down
        the chips may interfer
    2. We can start with manipulating the objects because we know that 
    no obkects will be 20 cms from the back wall, so we can go all the way left and 
    pick up pedestals and move the ducks up so they can be stored in the recycle area
        pro: we will be starting off with the hardest/ main task of the competition
        con: due to time we may lose out on easy points
'''

'''
So we would start off by moving all the way left on the game board
    the sensors to do that would be ultrasonic sensor 2 and 3
    we would want it to stop once it touches the wall or a little before the wall
'''
start_pos = range.getRight()
while start_pos > 15: # value should be adjustable to make sure bot does not hit the wall
    command.goRight()

'''
We will then run the objectTracking algorithm where the robot will approch the objects:
    pedestals are going to be approached with a higher priority
    once we approach a pedestal the robot will run the TOF sesnor code to aligin itself
    with the pedestal

    once the pedestal is stored we will want the robot to go back to the back wall
    using the ultrasonic sensors 4 and 5. Once it is on the back wall, we want to move the 
    robot to the right a little so it can counteract any adjustments made when the robot
    was alighning itself with a pedestal. This will also ensure any ducks are in the correct range
    for the last push
'''
# make a function in Robot.py that will move the robot up and down plus alighn itself after
# it adjusts to pick up an object
def pickupPath(self):

        while not bot.rng.getLeft(0) <= 10 and not bot.rng.getLeft(1) <= 10:
            while not bot.rng.getLeft(0) <= 10:
                if bot.realSense.getObjDetect(0) == 1:
                    break
                bot.ctrl.goLeft(0.5)
            
            bot.ctrl.stopBot()

            if bot.rng.getObjDetect()[0] == 1:
                bot.cameraAlign()
                bot.tofApproach()
                bot.tofAllign()
                bot.pickUprightPedestal() # picks up the pedestal
                while not bot.rng.getBack(0) <= 30 and not bot.rng.getBack(1) <= 30:
                    bot.ctrl.goBackwards(0.5)
                bot.ctrl.stopBot()
            bot.ctrl.goRight(1)
'''
We will repeat this process until the ultrasonic sensors 6 and 7 are a low range.
we will keep on moving left until the ultrasonic sensors reach that range.
'''
# while loop the above function that will be made
while range.getLeft() > 10:
    pickupPath(self)

'''
Assuming we have all pedestals stored on board we will go to the top wall using
ultrasonic sensors 0 and 1 until they are a low value. Then rotate the robot. Will then
make the robot move forward for a certain duration.

It would make sense to move forward a little bit then move right just incase any ducks are starting
to move out the pushing range, then align itself again

Definetly worth seeing if the robot can push all the ducks if theyr within the pushing range

By pushing range i mean where the robot can push the ducks in one push and not leaving
a duck behind
'''
# a while loop that will push the ducks
while range.getFront() > 10:
    command.goFoward(self,0.5)
command.rotateRight(self,1)
i = 0
while i not 10: 
    command.goFoward(self,0.5)
    i += 1
### ESSENTIALLY MILESTONE 1 and 2 SHOULD BE COMPLETED ABOVE ###

'''
Once we push all the ducks in the recycle area, we can assume our location on the gameboard will
be at the left side of the top recycle area

In this moment we will rotate the robot left and use the ultrasonic sensors 6 and 7 to approach
the red/green animal pen on the top left

drop off the correct color chips using the color sensors

use ultrasonic sensors 4 and 5 to move to the back wall and use the color sensor to identify the pen color

drop off the correct color chips using the color sensors

move the the robot until it returns to the start box
'''

'''
now that gameboard should be relatively cleared
all pedestals on board and all ducks in the top recycle area

we can now move to the first statue we want to build

return to start area

move to second statue area we want to build

return to start area

move to third statue area (would want this to be the statue area near fire works)
'''

### ESSENTIALLY MILESTONE 3 SHOULD BE COMPLETED ABOVE ###

'''
activate fireworks
'''

# Risk may be time so if want to build as we pick up pedestal
# it can definitlly help but adds a risk of running into built statues
# takes away the leansy of having open space