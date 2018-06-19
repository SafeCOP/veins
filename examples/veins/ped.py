#!/usr/bin/env python
"""
@file    runner.py
@author  Lena Kalleske
@author  Daniel Krajzewicz
@author  Michael Behrisch
@author  Jakob Erdmann
@date    2009-03-26
@version $Id: runner.py 22608 2017-01-17 06:28:54Z behrisch $

Tutorial for traffic light control via the TraCI interface.
This scenario models a pedestrian crossing which switches on demand.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2009-2017 DLR/TS, Germany

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random


# the directory in which this script resides
THISDIR = os.path.dirname(__file__)


# we need to import python modules from the $SUMO_HOME/tools directory
# If the the environment variable SUMO_HOME is not set, try to locate the python
# modules relative to this script
try:
    # tutorial in tests
    sys.path.append(os.path.join(THISDIR, '..', '..', '..', '..', "tools"))
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        THISDIR, "..", "..", "..")), "tools"))  # tutorial in docs

    import traci
    from sumolib import checkBinary
    import randomTrips
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

# minimum green time for the vehicles
MIN_GREEN_TIME = 15
# the first phase in tls plan. see 'pedcrossing.tll.xml'
VEHICLE_GREEN_PHASE = 0
# the id of the traffic light (there is only one). This is identical to the
# id of the controlled intersection (by default)
TLSID = 'C'

# pedestrian edges at the controlled intersection
WALKINGAREAS = [':C_w0', ':C_w1']
CROSSINGS = [':C_c0']

#PROGETTO_VARIABILI_GLOBALI
numeroped = 0  # random.randint(1,8)
deltaCriticalTime = (random.randint(0,8) * 1000)


def run():
    """execute the TraCI control loop"""
    # track the duration for which the green phase of the vehicles has been
    # active
    greenTimeSoFar = 0

    # whether the pedestrian button has been pressed
    activeRequest = False

    #PROGETTO_VARIABILI_LOCALI
    progettoboolean = False
    print ("number of ped "+ str(numeroped))
    print ("delta critical time " + str(deltaCriticalTime))
    time=0
    timeCritic=0

    # main loop. do something every simulation step until no more vehicles are
    # loaded or running
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        #PROGETTO 
        if progettoboolean:
            if (traci.simulation.getCurrentTime() == timeCritic): 
                progetto2()

        # decide wether there is a waiting pedestrian and switch if the green
        # phase for the vehicles exceeds its minimum duration
        if not activeRequest:
            activeRequest = checkWaitingPersons()
        if traci.trafficlights.getPhase(TLSID) == VEHICLE_GREEN_PHASE:
            greenTimeSoFar += 1
            if greenTimeSoFar > MIN_GREEN_TIME:
                # check whether someone has pushed the button

                if activeRequest:
                    # switch to the next phase
                    traci.trafficlights.setPhase(
                        TLSID, VEHICLE_GREEN_PHASE + 1)
                    # reset state
                    activeRequest = False
                    greenTimeSoFar = 0
                    
                    #PROGETTO
                    try:
                        if (traci.person.getNextEdge("ped"+ str(numeroped)) in CROSSINGS):
                            time = traci.simulation.getCurrentTime()
                            print ("time ped in crossing "+ str(time))
                            progettoboolean = True
                            timeCritic = time + deltaCriticalTime + 5000
                            print("time critical " + str(timeCritic))
                    except:
                        print ("nessun ped"+ str(numeroped))

    sys.stdout.flush()
    traci.close()

#FUNZIONI_PROGETTO                                         #ped1 viene creato a 29000
def progetto2():                                           #ped1 arriva a 61000   #tempo critico per ped1: da 66000 a 74000 
    print ("progetto2" + "->ped"+ str(numeroped))          #deltaCriticTime = 8000
    traci.person.setSpeed("ped"+ str(numeroped), 0)        #deltaToCriticalTime = 37000
    progettoboolean = False                                #per scrivere un intero concatenato su una stringa si usa str(int))


def progetto1(): 
    time = traci.simulation.getCurrentTime()
    print (time)
    return time

def progetto():
    if (traci.person.getNextEdge("ped1") in CROSSINGS):
       print ("progetto")
       time = traci.simulation.getCurrentTime()
       print (time)
#FINE

def checkWaitingPersons():
    """check whether a person has requested to cross the street"""

    # check both sides of the crossing
    for edge in WALKINGAREAS:
        peds = traci.edge.getLastStepPersonIDs(edge)
        # check who is waiting at the crossing
        # we assume that pedestrians push the button upon
        # standing still for 1s
        for ped in peds:
            if (traci.person.getWaitingTime(ped) == 1 and
                    traci.person.getNextEdge(ped) in CROSSINGS):
                print("%s pushes the button" % ped)
                return True
    return False



# this is the main entry point of this script
if __name__ == "__main__":
    #dir_path3 = os.path.realpath("/mnt/pietro/Programmi/veins-veins-4.5/examples/veins/run.sumocfg")
    # load whether to run with or without GUI
    #options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    # generate the pedestrians for this simulation
    #randomTrips.main(randomTrips.get_options([
     #   '--net-file', net,
      #  '--output-trip-file', 'pedestrians.trip.xml',
       # '--seed', '42',  # make runs reproducible
       # '--pedestrians',
       # '--prefix', 'ped',
        # prevent trips that start and end on the same edge
       # '--min-distance', '1',
       # '--trip-attributes', 'departPos="random" arrivalPos="random"',
       # '--binomial', '4',
       # '--period', '35']))

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    #traci.start(['sumo', '-c', dir_path3])
    run()
