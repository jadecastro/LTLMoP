RobotName: # Robot Name
Unicycle Simulation

Type: # Robot type
unicycleSim

ActuatorHandler: # Robot default actuator handler with default argument values

DriveHandler: # Robot default drive handler with default argument values
share.Drive.UnicycleDriveHandler(multiplier=50.0,maxspeed=999.0)

InitHandler: # Robot default init handler with default argument values
unicycleSim.UnicycleSimInitHandler()

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
unicycleSim.UnicycleSimLocomotionCommandHandler(speed=1.0)

MotionControlHandler: # Robot default motion control handler with default argument values
share.MotionControl.ReasynsFastHandler(scalingPixelsToMeters=1.,fname='reasyns_primitives')

PoseHandler: # Robot default pose handler with default argument values
unicycleSim.UnicycleSimPoseHandler()

SensorHandler: # Robot default sensor handler with default argument values



