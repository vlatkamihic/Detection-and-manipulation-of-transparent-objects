#!/usr/bin/env python3
import rospy
from cleargrasp.srv import CheckCurrentPhase, CheckCurrentPhaseResponse

current_phase = 0

def handle_current_phase(req):
    global current_phase
    isMyTurn = False
    if req.my_phase == current_phase and req.isFinished is not True:
        isMyTurn = True
        print("current phase"+str(current_phase))
    elif req.my_phase == current_phase and req.isFinished == True:
        isMyTurn = False
        current_phase += 1
    return CheckCurrentPhaseResponse(isMyTurn)

def check_current_phase_server():
    rospy.init_node('check_current_phase_server')
    s = rospy.Service('check_current_phase', CheckCurrentPhase, handle_current_phase)
    rospy.spin()

if __name__ == "__main__":
    check_current_phase_server()
