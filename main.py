import time as clock
from datetime import *
import matplotlib.pyplot as plt
import random

def main():
    kp = .5  # proportion constant	
    ki = .1  # integrated constant	
    kd = .2 # dirivitave constant; rate of change

    sensitivity   = .1     #how close you want the value to get
    sensitivityEnabled = False #do you want to use sensitivity?
    length        = 30
    timeStep      = 1    #speed of time for the simulation
    currentPoint  = 1    #the current position
    desiredPoint  = 20   #the goal


    maxForce      = 2    #maximum external force

    tElap=0              
    lastTelap = 0
    externalForce = 0
    cuError = 0
    error = 0
    rateError = 0
    lastError = 0
    pointHistory = []
    cycles = 0
    while(cycles < length):
        print("Cycle "+str(cycles)+" of "+ str(length))
        cycles+=1
        #time
        lastTelap = tElap
        clock.sleep(timeStep)
        tElap += timeStep
    
        externalForce = random.randrange(0, maxForce+1)
        

        #PID Controller
        error = desiredPoint-currentPoint                              # P

        cuError = cuError + (error*(tElap-lastTelap))                  # I

        rateError = rateError = (error-lastError)/(tElap-lastTelap)    # D

        out = kp*error + ki*cuError + kd*rateError                     # Combined values

        currentPoint+= out
        currentPoint+= externalForce
        pointHistory.append(currentPoint)
        lastError = error
        if desiredPoint-currentPoint < sensitivity and sensitivityEnabled == True:
            break
    
    #Save it as a plot
    plt.plot(pointHistory)
    plt.savefig(str(datetime.now())+".png")


main()
    
