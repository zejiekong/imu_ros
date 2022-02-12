import numpy as np
import math

D2R = math.pi/180

# low accuracy, from AHRS380
#http://www.memsic.cn/userfiles/files/Datasheets/Inertial-System-Datasheets/AHRS380SA_Datasheet.pdf
gyro_low_accuracy = {'b': np.array([0.0, 0.0, 0.0]) * D2R,
                     'b_drift': np.array([10.0, 10.0, 10.0]) * D2R/3600.0,
                     'b_corr':np.array([100.0, 100.0, 100.0]),
                     'arw': np.array([0.75, 0.75, 0.75]) * D2R/60.0}
accel_low_accuracy = {'b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
                      'b_drift': np.array([2.0e-4, 2.0e-4, 2.0e-4]),
                      'b_corr': np.array([100.0, 100.0, 100.0]),
                      'vrw': np.array([0.05, 0.05, 0.05]) / 60.0}

# mid accuracy, partly from IMU381
gyro_mid_accuracy = {'b': np.array([0.0, 0.0, 0.0]) * D2R,
                     'b_drift': np.array([3.5, 3.5, 3.5]) * D2R/3600.0,
                     'b_corr':np.array([100.0, 100.0, 100.0]),
                     'arw': np.array([0.25, 0.25, 0.25]) * D2R/60}
accel_mid_accuracy = {'b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
                      'b_drift': np.array([5.0e-5, 5.0e-5, 5.0e-5]),
                      'b_corr': np.array([100.0, 100.0, 100.0]),
                      'vrw': np.array([0.03, 0.03, 0.03]) / 60}

# high accuracy, partly from HG9900, partly from
# http://www.dtic.mil/get-tr-doc/pdf?AD=ADA581016
gyro_high_accuracy = {'b': np.array([0.0, 0.0, 0.0]) * D2R,
                      'b_drift': np.array([0.1, 0.1, 0.1]) * D2R/3600.0,
                      'b_corr':np.array([100.0, 100.0, 100.0]),
                      'arw': np.array([2.0e-3, 2.0e-3, 2.0e-3]) * D2R/60}
accel_high_accuracy = {'b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
                       'b_drift': np.array([3.6e-6, 3.6e-6, 3.6e-6]),
                       'b_corr': np.array([100.0, 100.0, 100.0]),
                       'vrw': np.array([2.5e-5, 2.5e-5, 2.5e-5]) / 60}

IMU_dict = {"low":{"accel":accel_low_accuracy,"gyro":gyro_low_accuracy},
            "mid":{"accel":accel_mid_accuracy,"gyro":gyro_mid_accuracy},
            "high":{"accel":accel_high_accuracy,"gyro":gyro_high_accuracy}}


    