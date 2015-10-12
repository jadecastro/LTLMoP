#!/usr/bin/env python
"""
=======================================================
holonomicDrive.py - Ideal Holonomic Point Drive Handler
=======================================================

Passes velocity requests directly through.
Used for ideal holonomic point robots.
"""

import lib.handlers.handlerTemplates as handlerTemplates

class YoubotHolonomicDriveHandler(handlerTemplates.DriveHandler):
    def __init__(self, executor, shared_data,multiplier):
        """
        Passes velocity requests directly through.
        Used for ideal holonomic point robots.

        multiplier (float): Scale the velocity from motionControlHandler (default=1.0,min=1.0,max=50.0)
        """
        try:
            #self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)
            self.coordmap = executor.hsub.coordmap_lab2map
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

        self.mul = multiplier
        self.maxspeed = 0.1
        self.maxangle = 2

    def setVelocity(self, vx, vy, w, theta=0):
        vx = min(vx*self.mul,self.maxspeed)
        vy = min(vy*self.mul,self.maxspeed)
        w = max(min(w,self.maxangle),-self.maxangle)

        #print "VEL:%f,%f" % tuple(self.coordmap([x, y]))
        # print 'Drive handler inputs x:' + str(x) + ' y:' + str(y)
        self.loco.sendCommand([vx,vy,w])

