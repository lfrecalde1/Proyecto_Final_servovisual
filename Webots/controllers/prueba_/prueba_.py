from controller import Supervisor
from controller import Node
import sys

TIME_STEP = 32

supervisor = Supervisor()

# do this once only
robot_node = supervisor.getFromDef("mavic")

if robot_node == None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("translation")
velocidad = robot_node.getField("Velocity")
while supervisor.step(TIME_STEP) != -1:
    # this is done repeatedly
    values = trans_field.getSFVec3f()
  
    print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))