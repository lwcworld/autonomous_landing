from pykalman import UnscentedKalmanFilter
import numpy as np
ukf = UnscentedKalmanFilter(lambda x, w: x + np.sin(w), lambda x, v: x + v, observation_covariance=0.1)
(filtered_state_means, filtered_state_covariances) = ukf.filter([0, 1, 2])
(filtered_state_means, filtered_state_covariances) = ukf.filter([0, 1, 2])

(filtered_state_means, filtered_state_covariances) = ukf.filter([0, 1, 2])

print(filtered_state_means)
