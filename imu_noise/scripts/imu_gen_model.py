import numpy as np
import math
import imu_model_data

class imu_model:
    def __init__(self,accuracy):
        self.accel_err = imu_model_data.IMU_dict[accuracy]["accel"]
        self.gyro_err =  imu_model_data.IMU_dict[accuracy]["gyro"]
        self.fs = 100

        self.old_accel_bias_drift = np.array([0,0,0])
        self.old_gyro_bias_drift = np.array([0,0,0])
        self.old_accel_drift_noise = np.array([0,0,0])
        self.old_gyro_drift_noise = np.array([0,0,0])

        self.a_accel = np.array([0,0,0])
        self.b_accel = np.array([0,0,0])

        self.a_gyro = np.array([0,0,0])
        self.b_gyro = np.array([0,0,0])

    def accel_gen(self, ref_a):
        dt = 1.0/self.fs
        ## simulate sensor error
        # static bias
        accel_bias = self.accel_err['b']
        # bias drift
        accel_bias_drift,self.old_accel_drift_noise = self.bias_drift("accel",self.accel_err['b_corr'], self.accel_err['b_drift'],self.old_accel_bias_drift,self.old_accel_drift_noise)

        # accelerometer white noise
        accel_noise = np.random.randn(3)
        accel_noise[0] = self.accel_err['vrw'][0] / math.sqrt(dt) * accel_noise[0]
        accel_noise[1] = self.accel_err['vrw'][1] / math.sqrt(dt) * accel_noise[1]
        accel_noise[2] = self.accel_err['vrw'][2] / math.sqrt(dt) * accel_noise[2]
        # true + constant_bias + bias_drift + noise
        a_mea = ref_a + accel_bias + accel_bias_drift + accel_noise  #+ acc_vib
        self.old_accel_bias_drift = accel_bias_drift #store current bias drift for next reading
        return a_mea

    def gyro_gen(self, ref_w):
        
        dt = 1.0/self.fs
        ## simulate sensor error
        # static bias
        gyro_bias = self.gyro_err['b']

        # bias drift
        gyro_bias_drift,self.old_gyro_drift_noise = self.bias_drift("gyro",self.gyro_err['b_corr'], self.gyro_err['b_drift'], self.old_gyro_bias_drift,self.old_gyro_drift_noise)

        # gyroscope white noise
        gyro_noise = np.random.randn(3)

        gyro_noise[0] = self.gyro_err['arw'][0] / math.sqrt(dt) * gyro_noise[0]
        gyro_noise[1] = self.gyro_err['arw'][1] / math.sqrt(dt) * gyro_noise[1]
        gyro_noise[2] = self.gyro_err['arw'][2] / math.sqrt(dt) * gyro_noise[2]
        # true + constant_bias + bias_drift + noise
        w_mea = ref_w + gyro_bias + gyro_bias_drift + gyro_noise  # + gyro_vib
        self.old_gyro_bias_drift = gyro_bias_drift
        return w_mea

    def bias_drift(self,sensor_type,corr_time, drift,old_sensor_bias_drift,old_drift_noise):
        a_b_dict = {"accel":{"a":self.a_accel,"b":self.b_accel},"gyro":{"a":self.a_gyro,"b":self.b_gyro}}
        a = a_b_dict[sensor_type]["a"]
        b = a_b_dict[sensor_type]["b"]
        # 3 axis
        sensor_bias_drift = np.zeros((3))
        for i in range(0, 3):
            if not math.isinf(corr_time[i]):

                #sensor_bias_drift[0, :] = np.random.randn(3) * drift
                drift_noise = np.random.randn(3)
                #for j in range(1, n):
                sensor_bias_drift[i] = a[i]*old_sensor_bias_drift[i] + b[i]*old_drift_noise[i]
            else:
                # normal distribution
                sensor_bias_drift[i] = drift[i] * np.random.randn(1)
        return sensor_bias_drift,drift_noise

    def set_sensor_bias_param(self):
        # only called once when receiving first reading
        params_dict = {"accel":{"corr":self.accel_err['b_corr'], "drift":self.accel_err['b_drift'],"a":self.a_accel,"b":self.b_accel},
                        "gyro":{"corr":self.accel_err['b_corr'], "drift":self.accel_err['b_drift'],"a":self.a_gyro,"b":self.b_gyro}}
        for i in params_dict:
            corr_time = params_dict[i]["corr"]
            drift = params_dict[i]["drift"]
            a = params_dict[i]["a"]
            b = params_dict[i]["b"]
            
            # First-order Gauss-Markov
            a[0] = 1 - 1/self.fs/corr_time[0]
            a[1] = 1 - 1/self.fs/corr_time[1]
            a[2] = 1 - 1/self.fs/corr_time[2]

            # For the following equation, see issue #19 and
            # https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3812568/ (Eq. 3).
            b[0] = drift[0] * np.sqrt(1.0 - np.exp(-2/(self.fs * corr_time[0])))
            b[1] = drift[1] * np.sqrt(1.0 - np.exp(-2/(self.fs * corr_time[1])))
            b[2] = drift[2] * np.sqrt(1.0 - np.exp(-2/(self.fs * corr_time[2])))

# if __name__ == "__main__":
#     Imu = imu_model("high")
#     Imu.set_sensor_bias_param()
#     print(Imu.accel_gen(np.array([1,1,1]))) #input is 1d array
#     print(Imu.accel_gen(np.array([1,1,1])))
#     print(Imu.accel_gen(np.array([1,1,1])))
