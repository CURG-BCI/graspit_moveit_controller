

import random
import itertools
from numpy import dot
from numpy import array, unique
from math import acos
from sklearn import neighbors

import tf_conversions.posemath as pm


class DemonstrationPoseAnalyzer():

    def __init__(self):
        self.data = set()
        self.retrain_threshold = 1
        self.model = neighbors.NearestNeighbors(n_neighbors=20, algorithm='kd_tree')
        self.max_data_len = 5000
        self.array_data = []
        self.grasp_classes = []

    def data_to_array(self):
        self.array_data = array([pm.toMatrix(pm.fromMsg(grasp_msg[0].final_grasp_pose))[:3, 3]
                             for grasp_msg in self.data])

    def train_model(self, grasp, grasp_class):

        self.data.add((grasp, grasp_class))
        if len(self.data) > self.max_data_len:
            self.sparcify_data()

        if not len(self.data) % self.retrain_threshold:
            self.data_to_array()
            self.model.fit(self.array_data)

    def sparcify_data(self):
        self.data = random.sample(self.data, len(self.data)/2.0)

    def analyze_pose(self, demo_pose):

        if len(self.data) == 0:
            return 1.0

        demo_pose_matrix = pm.toMatrix(pm.fromMsg(demo_pose))
        demo_position = demo_pose_matrix[:3, 3]

        distances, indices = self.model.kneighbors(demo_position)
        indices = unique(indices)
        nbrs = [t for t in itertools.compress(self.data, indices)]
        valid_nbrs = []
        for n in nbrs:
            pose = pm.toMatrix(pm.fromMsg(n[0].final_grasp_pose))
            if acos(dot(pose[:3, 2], demo_pose_matrix[:3, 2]) < .52):
                valid_nbrs.append(n)

        if len(valid_nbrs):
            success_probability = len([n for n in valid_nbrs if n[1] & 1])/(1.0*len(valid_nbrs))
        else:

            success_probability = 0

        return success_probability