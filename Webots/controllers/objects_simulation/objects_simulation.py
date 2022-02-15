from controller import Supervisor
import sys

TIME_STEP = 32

supervisor = Supervisor()

# do this once only
robot_node = supervisor.getFromDef("objects")
obj_1 = supervisor.getFromDef("object_1")
if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("rotation")
trans_object_1 = obj_1.getField("translation")
while supervisor.step(TIME_STEP) != -1:
    # this is done repeatedly
    values = trans_field.getSFRotation()
    print("General: %g %g %g %g" % (values[0], values[1], values[2], values[3]))
    objec1_pos = trans_object_1.getSFVec3f()
    print("Obj_1: %g %g %g" % (objec1_pos[0], objec1_pos[1], objec1_pos[2]))
    