from numpy import eye, array, diag
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.ur.robot import *
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint

from dynamic_graph.sot.application.velocity.precomputed_tasks import Solver, createCenterOfMassFeatureAndTask, createOperationalPointFeatureAndTask, initializeSignals
from dynamic_graph.sot.dyninv import SolverDynReduced

class SolverUR:

    def __init__(self, robot, solverType=SOT):
        self.robot = robot

        # Make sure control does not exceed joint limits.
        self.jointLimitator = JointLimitator('joint_limitator')
        plug(self.robot.dynamic.position, self.jointLimitator.joint)
        plug(self.robot.dynamic.upperJl, self.jointLimitator.upperJl)
        plug(self.robot.dynamic.lowerJl, self.jointLimitator.lowerJl)

        # Create the solver.
        self.sot = solverType('solver')
        self.sot.signal('damping').value = 1e-6
        self.sot.setSize(self.robot.dimension)


        # Plug the solver control into the filter.
         #plug(self.sot.control, self.jointLimitator.controlIN)

        # Important: always use 'jointLimitator.control'
        # and NOT 'sot.control'!

        if robot.device:
            plug(self.sot.forces, robot.device.control)

    def push (self, task):
        """
        Proxy method to push a task in the sot
        """
        self.sot.push (task.name)

    def remove (self, task):
        """
        Proxy method to remove a task from the sot
        """
        self.sot.remove (task.name)

    def __str__ (self):
        return self.sot.display ()

def initialize (robot, solverType=SOT):
    """
    Initialize the solver, and define shortcuts for the operational points
    """

    # TODO: this should disappear in the end.     
    # --- center of mass ------------
    (robot.featureCom, robot.featureComDes, robot.comTask, robot.comGain) = \
        createCenterOfMassFeatureAndTask(robot,
        '{0}_feature_com'.format(robot.name),
        '{0}_feature_ref_com'.format(robot.name),
        '{0}_task_com'.format(robot.name))

    # --- operational points tasks -----
    robot.features = dict()
    robot.tasks = dict()
    robot.gains = dict()
    for op in robot.OperationalPoints:
        (robot.features[op], robot.tasks[op], robot.gains[op]) = \
            createOperationalPointFeatureAndTask(robot,
            op, '{0}_feature_{1}'.format(robot.name, op),
            '{0}_task_{1}'.format(robot.name, op))
        # define a member for each operational point
        w = op.split('-')
        memberName = w[0]
        for i in w[1:]:
            memberName += i.capitalize()
        setattr(robot, memberName, robot.features[op])

    initializeSignals (robot, robot)

    # --- create solver --- #
    solver = SolverUR (robot, SolverDynReduced)

    # --- push balance task --- #
    metaContact = UrContactTask(robot)
    robot.tasks ['contact'] = metaContact.task
    robot.features ['contact'] = metaContact.feature
    metaContact.feature.selec.value = '011100'
    metaContact.featureDes.position.value = \
      array([[1,0,0,0],[0,1,0,0],[0,0,1,0.051],[0,0,0,1]])
    solver.push(robot.tasks['contact'])
#    solver.sot.addContact(robot.tasks['contact'])

    return solver


# -- HANDS ------------------------------------------------------------------
# TODO: directly use the feature.
def UrRightHandTask(robot):
    task=MetaTaskKine6d('right-wrist',robot.dynamic,'right-wrist','right-wrist')
    task.feature.frame('desired')
    return task

def UrLeftHandTask(robot):
    task=MetaTaskKine6d('left-wrist',robot.dynamic,'left-wrist','left-wrist')
    task.feature.frame('desired')
    return task

""" Create a task that aims at reducing the contribution of the some of the joints."""
def UrWeight(robot):
  # --- TASK POSTURE --------------------------------------------------
  # set a default position for the joints.
  feature = FeatureGeneric('feature_weight')
  weight = 18 * (0,) + (500,)* 1 + (51-19) * (1,)
  feature.jacobianIN.value = diag(weight)
  feature.errorIN.value = 51 * (0,)

  task=Task('weight')
  task.add('feature_weight')

  gainWeight = GainAdaptive('gain_weight')
  gainWeight.set(0.1,0.1,125e3)
  gainWeight.gain.value = 5
  plug(task.error, gainWeight.error)
  plug(gainWeight.gain, task.controlGain)

  return (task, feature)

def initPostureTask(robot):
  # --- TASK POSTURE --------------------------------------------------
  # set a default position for the joints.
  robot.features['featurePosition'] = FeaturePosture('featurePosition')
  plug(robot.device.state,robot.features['featurePosition'].state)
  robotDim = len(robot.dynamic.velocity.value)
  robot.features['featurePosition'].posture.value = robot.halfSitting

  postureTaskDofs = [True]*6 + [False]*(51-6)
  postureTaskDofs = [True]*(51)

  for dof,isEnabled in enumerate(postureTaskDofs):
    if dof > 6:
      robot.features['featurePosition'].selectDof(dof,isEnabled)

  robot.tasks['robot_task_position']=Task('robot_task_position')
  robot.tasks['robot_task_position'].add('featurePosition')
  # featurePosition.selec.value = toFlags((6,24))

  gainPosition = GainAdaptive('gainPosition')
  gainPosition.set(0.1,0.1,125e3)
  gainPosition.gain.value = 5
  plug(robot.tasks['robot_task_position'].error,gainPosition.error)
  plug(gainPosition.gain,robot.tasks['robot_task_position'].controlGain)

# -- JOINTS LIMITS  ---------------------------------------------------------

def UrJointLimitsTask(robot,dt):
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    task = TaskJointLimits('taskJL')
    plug(robot.dynamic.position,task.position)
    task.controlGain.value = 10
    task.referenceInf.value = robot.dynamic.lowerJl.value
    task.referenceSup.value = robot.dynamic.upperJl.value
    task.dt.value = dt
    task.selec.value = toFlags(range(18,25)+range(26,27)+range(28,31)+range(32,40)+range(41,42)+range(43,46)+range(47,50))
    return task
    

# -- CONTACT  --------------------------------------------------------------- 
def UrContactTask(robot):
    task = MetaTaskKine6d('contact',robot.dynamic,'contact','left-ankle')
    task.feature.frame('desired')
    task.feature.selec.value = '111111'
    task.featureDes.position.value = \
      array([[1,0,0,0],[0,1,0,0],[0,0,1,0.051],[0,0,0,1]])
    task.gain.setConstant(10)
    #locals()['contact'] = task
    return task

# -- WAIST  ------------------------------------------------------------------ 
def UrFixedTask(robot):
    task = MetaTaskKine6d('base',robot.dynamic,'waist','waist')
    baseMground=eye(4);
    baseMground[0:3,3] = (0,0,0)
    task.opmodif = matrixToTuple(baseMground)
    task.feature.frame('desired')
    task.feature.selec.value = '100011'
    task.gain.setConstant(10)
    gotoNd(task,(0,0,0,0,0,0),'100011',(4.9,0.9,0.01,0.9))
    return task

    
__all__ = ["UrRightHandTask", "UrLeftHandTask",
            "UrJointLimitsTask", "UrContactTask", 
            "initialize", "UrWeight",
            "UrFixedTask", "initPostureTask"]