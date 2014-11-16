import pymatlab
import numpy as np
import math

# from numpy.random import randn

ses = pymatlab.session_factory()

# a = randn(20,10,30)
# ses.putvalue('A',a)
# ses.run('B=2*A')
# b = ses.getvalue('B')
# print b
# print (2*a==b).all()


i = 1
goalVelocity = [0, 2]  # temporarily setting this to zero
# ses.putvalue('zGoal{'+str(i)+'}',np.float_(np.mat(goalVelocity).T))
ses.putvalue('zGoalNew',np.float_(goalVelocity))
ses.run('zGoal=[];')
ses.run('zGoal=[zGoal zGoalNew zGoalNew+1];')
ses.run('zGoalCell=mat2cell(zGoal,2,[1,1])')
print('Set goalVelocity completed')
print("in python: " + str(np.float_(goalVelocity)))
print("in MATLAB: " + str(ses.getvalue('zGoal')))
# print("in MATLAB: " + str(ses.getvalue('zGoal{'+str(i)+'}')))

ses.run('zGoal_robot1=zGoal(:,1);')
ses.run('zGoal_robot2=zGoal(:,2);')
print("  robot1: " + str(ses.getvalue('zGoal_robot1')))
print("  robot2: " + str(ses.getvalue('zGoal_robot2')))

del ses

#ses.run('run /home/jon/Dropbox/Repos/uav-avoidance/multiquad-sim/simQuads')
