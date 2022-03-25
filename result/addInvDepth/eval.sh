



python ../../evaluation/evaluate_ate_scale.py  ../gt/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv  CameraTrajectory-MH01.txt
python ../../evaluation/evaluate_ate_scale.py  ../gt/MH_02_easy/mav0/state_groundtruth_estimate0/data.csv  CameraTrajectory-MH02.txt
python ../../evaluation/evaluate_ate_scale.py  ../gt/MH_03_medium/mav0/state_groundtruth_estimate0/data.csv  CameraTrajectory-MH03.txt
python ../../evaluation/evaluate_ate_scale.py  ../gt/MH_04_difficult/mav0/state_groundtruth_estimate0/data.csv  CameraTrajectory-MH03.txt
python ../../evaluation/evaluate_ate_scale.py  ../gt/MH_05_difficult/mav0/state_groundtruth_estimate0/data.csv  CameraTrajectory-MH04.txt

python ../../evaluation/evaluate_ate_scale.py  ../gt/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv  CameraTrajectory-V101.txt 
python ../../evaluation/evaluate_ate_scale.py  ../gt/V1_02_medium/mav0/state_groundtruth_estimate0/data.csv  CameraTrajectory-V102.txt 
python ../../evaluation/evaluate_ate_scale.py  ../gt/V1_03_difficult/mav0/state_groundtruth_estimate0/data.csv  CameraTrajectory-V103.txt 

# # InvDepth 2
# 0.088337,1.019389,0.034470
# 0.043218,0.993443,0.035937
# 0.049683,1.005531,0.045443
# Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?
# Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?
# 0.040773,1.011539,0.034901
# 0.015033,1.001505,0.014786
# 0.078891,1.016067,0.074877


# # InvDepth 1
# 0.063672,1.009765,0.048389
# 0.051332,0.992853,0.044107
# 0.040601,0.998021,0.039955
# NULL
# NULL
# 0.035679,1.009502,0.031151
# 0.019348,1.004098,0.017926
# 0.055699,1.023327,0.042910


#2 

# 0.070665,1.017965,0.029329
# 0.057446,0.989706,0.044021
# 0.055311,0.989040,0.037293
# Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?
# Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?
V101:
# 0.039321,1.011331,0.033428
# 0.024916,1.002094,0.024629
# 0.056523,1.021116,0.046464


# 1 : 

# 0.030633,1.002593,0.028558
# 0.089421,0.980775,0.049181
# 0.067291,0.992937,0.061915
# Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?
# Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?
# 0.033547,1.004886,0.032322
# 0.020125,1.009161,0.011984
# 0.052199,1.017829,0.044352




# /*

# compared_pose_pairs 3545 pairs
# absolute_translational_error.rmse 0.063672 m
# absolute_translational_error.mean 0.051072 m
# absolute_translational_error.median 0.051259 m
# absolute_translational_error.std 0.038025 m
# absolute_translational_error.min 0.008981 m
# absolute_translational_error.max 1.250060 m
# max idx: 4
# compared_pose_pairs 3545 pairs
# absolute_translational_error.rmse 0.063672 m
# absolute_translational_errorGT.rmse 0.048389 m
# 0.063672,1.009765,0.048389
# compared_pose_pairs 2156 pairs
# absolute_translational_error.rmse 0.051332 m
# absolute_translational_error.mean 0.048267 m
# absolute_translational_error.median 0.047188 m
# absolute_translational_error.std 0.017473 m
# absolute_translational_error.min 0.011879 m
# absolute_translational_error.max 0.197233 m
# max idx: 7
# compared_pose_pairs 2156 pairs
# absolute_translational_error.rmse 0.051332 m
# absolute_translational_errorGT.rmse 0.044107 m
# 0.051332,0.992853,0.044107
# compared_pose_pairs 2268 pairs
# absolute_translational_error.rmse 0.040601 m
# absolute_translational_error.mean 0.035963 m
# absolute_translational_error.median 0.031804 m
# absolute_translational_error.std 0.018844 m
# absolute_translational_error.min 0.001136 m
# absolute_translational_error.max 0.170834 m
# max idx: 4
# compared_pose_pairs 2268 pairs
# absolute_translational_error.rmse 0.040601 m
# absolute_translational_errorGT.rmse 0.039955 m
# 0.040601,0.998021,0.039955
# Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?
# Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?
# compared_pose_pairs 2783 pairs
# absolute_translational_error.rmse 0.035679 m
# absolute_translational_error.mean 0.033604 m
# absolute_translational_error.median 0.033596 m
# absolute_translational_error.std 0.011990 m
# absolute_translational_error.min 0.001883 m
# absolute_translational_error.max 0.112730 m
# max idx: 4
# compared_pose_pairs 2783 pairs
# absolute_translational_error.rmse 0.035679 m
# absolute_translational_errorGT.rmse 0.031151 m
# 0.035679,1.009502,0.031151
# compared_pose_pairs 1504 pairs
# absolute_translational_error.rmse 0.019348 m
# absolute_translational_error.mean 0.015291 m
# absolute_translational_error.median 0.014102 m
# absolute_translational_error.std 0.011855 m
# absolute_translational_error.min 0.001240 m
# absolute_translational_error.max 0.210407 m
# max idx: 25
# compared_pose_pairs 1504 pairs
# absolute_translational_error.rmse 0.019348 m
# absolute_translational_errorGT.rmse 0.017926 m
# 0.019348,1.004098,0.017926
# compared_pose_pairs 1879 pairs
# absolute_translational_error.rmse 0.055699 m
# absolute_translational_error.mean 0.049816 m
# absolute_translational_error.median 0.045344 m
# absolute_translational_error.std 0.024915 m
# absolute_translational_error.min 0.004371 m
# absolute_translational_error.max 0.165131 m
# max idx: 34
# compared_pose_pairs 1879 pairs
# absolute_translational_error.rmse 0.055699 m
# absolute_translational_errorGT.rmse 0.042910 m
# 0.055699,1.023327,0.042910
