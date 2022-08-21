import csv
import numpy as np
import matlab.engine

def traj2mat(traj_dir):
	traj=[]
	with open(traj_dir, 'r') as file:
		csvreader = csv.reader(file)
		for row in csvreader:
			newrow = [0]
			for i in row:
				newrow.append(float(i))
			traj.append(newrow)
	return traj

eng = matlab.engine.start_matlab()
eng.cd('UOLA', nargout=0)
trajectories = [
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_4000.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_4001.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_4002.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_4003.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_4004.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_4005.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_4006.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_4008.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_4010.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_4012.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_5000.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_5001.csv',
# '/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_5002.csv',
'/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_6000.csv',
'/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_6004.csv',
'/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_6006.csv',
'/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_6008.csv',
'/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/training_data/traj_6010.csv',
'',
]
for num in range(101, 131):
	if num == 103:
		continue
	traj = traj2mat('/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/UOLA/training_data/traj_'+str(num)+'.csv')
	# print(traj.shape) 	
	eng.UOLA_learn('GMMtry.mat', matlab.double(traj), nargout=0)#matlab.double(traj), 'prediction.csv', nargout=2)