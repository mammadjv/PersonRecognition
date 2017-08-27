import numpy as np
import math
# ---------------------------
# input: list of 14 points of body
#        each point is [x,y,z]
# output: list of 14 soft biometric
#         features
# ---------------------------

class FeatureCombiner:
	# data[0] is list of points for all frames of person "0" 
	def __init__(self, data):
		self.data = data

    # mean_list[0] is list of features mean for person "0"
    # std_list[0] is list of features std for person "0"
	def train_joints(self):
		data = self.data
		mean_list = []
		std_list = []
		for each_data in data:
			each_mean,each_std = self.feature_properties(each_data)
			mean_list.append(each_mean)
			std_list.append(each_std)
		return mean_list, std_list
	# -------------------------------------
	def test_joints(self, points, mean_list, std_list):
		num_people = len(mean_list)
		sum_all = self.sum_people(points,mean_list,std_list)
		log_prob_all = []
		for i in range(num_people):
			tot = self.get_person_probability(points,mean_list[i],std_list[i])
			log_prob_all.append(tot - sum_all)
		max_person = np.argmax(log_prob_all)
		max_log = np.max(log_prob_all)
		return max_person,max_log
	# -------------------------------------
	def sum_people(self,points,mean_list,std_list):
		num_people = len(mean_list)
		sum_all = 0
		for i in range(num_people):
			soft_probability = self.get_feature_person(points,mean_list[0],std_list[0])
			this_probability = np.sum(np.log(soft_probability))
			sum_all += this_probability
		return sum_all
	# -------------------------------------
	def get_person_probability(self, points, features_mean, features_std):
		soft_probability = self.get_feature_person(points,features_mean,features_std)
		total_probability = np.sum(np.log(soft_probability))
		return total_probability
	# -------------------------------------
	def get_soft_features(self, points):
		V = []
		# V1
		V.append(np.linalg.norm(np.asarray(points[1])-np.asarray(points[0])))
		# V2
		V.append(np.linalg.norm(np.asarray(points[6])-np.asarray(points[2])))
		# V3
		V.append(np.linalg.norm(np.asarray(points[3])-np.asarray(points[2])))
		# V4
		V.append(np.linalg.norm(np.asarray(points[2])-np.asarray(points[1])))
		# V5
		V.append(np.linalg.norm(np.asarray(points[6])-np.asarray(points[1])))
		# V6
		V.append(np.linalg.norm(np.asarray(points[3])-np.asarray(points[1])))
		# V7
		V.append(np.linalg.norm(np.asarray(points[10])-np.asarray(points[9])))
		# V8
		V.append(np.linalg.norm(np.asarray(points[10])-np.asarray(points[9])))
		# V9
		V.append(np.linalg.norm(np.asarray(points[4])-np.asarray(points[3])) + np.linalg.norm(np.asarray(points[5])-np.asarray(points[4])))
		# V10
		V.append(np.linalg.norm(np.asarray(points[7])-np.asarray(points[6])) + np.linalg.norm(np.asarray(points[8])-np.asarray(points[7])))
		# V11
		V.append(np.linalg.norm(np.asarray(points[13])-np.asarray(points[12])) + np.linalg.norm(np.asarray(points[14])-np.asarray(points[13])))
		# V12
		V.append(np.linalg.norm(np.asarray(points[10])-np.asarray(points[9])) + np.linalg.norm(np.asarray(points[11])-np.asarray(points[10])))
		# V13
		V.append(np.linalg.norm(np.asarray(points[2])-np.asarray(points[1])) + np.linalg.norm(np.asarray(points[9])-np.asarray(points[2]))+
				np.linalg.norm(np.asarray(points[10])-np.asarray(points[9]))+np.linalg.norm(np.asarray(points[11])-np.asarray(points[10])))
		# V14
		V.append(np.linalg.norm(np.asarray(points[2])-np.asarray(points[1])) + np.linalg.norm(np.asarray(points[9])-np.asarray(points[2]))+
				np.linalg.norm(np.asarray(points[10])-np.asarray(points[9]))+np.linalg.norm(np.asarray(points[11])-np.asarray(points[10]))+
				np.linalg.norm(np.asarray(points[1])-np.asarray(points[0])))
		return V
	# ----------------------------------------------------------------------------------------------------------------------------------
	# input : list of points of all people
	# output: list of mean and std for every
	#         feature of all people
	# ---------------------------------------
	def feature_properties(self, list_points):
		points_features = []
		for points in list_points:
			points_features.append(self.get_soft_features(points))
		features_mean = np.mean(points_features,axis = 0)
		features_std = np.std(points_features,axis = 0)
		return features_mean,features_std
	# ----------------------------------------------------------------------------------------------------------------------------------
	def get_feature_person(self, points,features_mean,features_std):
		features = self.get_soft_features(points)
		probability = (np.exp((-(features - features_mean)**2)/(2*features_std**2)))/(features_std*math.sqrt(2*math.pi))
		return probability
