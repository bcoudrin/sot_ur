from dynamic_graph import plug
from dynamic_graph.sot.ur.ur_tasks import *
from dynamic_graph.ros import *
from dynamic_graph.sot.dyninv import SolverDynReduced

from time import sleep
plug(robot.device.state, robot.dynamic.position)
ros = Ros(robot)
sleep(2)
solver = initialize(robot)
robot.dynamic.velocity.value = robot.dimension*(0.,)
robot.dynamic.acceleration.value = robot.dimension*(0.,)
robot.dynamic.ffposition.unplug()
robot.dynamic.ffvelocity.unplug()
robot.dynamic.ffacceleration.unplug()
robot.dynamic.setProperty('ComputeBackwardDynamics','true')
robot.dynamic.setProperty('ComputeAccelerationCoM','true')
robot.device.control.unplug()
plug(solver.sot.control,robot.device.control)

dt = 0.001
taskRH = UrRightHandTask(robot)
taskLH = UrLeftHandTask(robot)
taskJL = UrJointLimitsTask(robot,dt)


from dynamic_graph.sot.core.meta_tasks_kine import gotoNd
targetRH = (0.5,-0.5,0.5)
gotoNd(taskRH,targetRH,'111',(4.9,0.9,0.01,0.9))
targetLH = (0.5,0.5,0.5)
gotoNd(taskLH,targetLH,'111',(4.9,0.9,0.01,0.9))

solver.push(taskRH.task)
solver.push(taskLH.task)
