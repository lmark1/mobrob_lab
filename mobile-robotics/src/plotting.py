import matplotlib.pyplot as plt

def read_from_csv_test(path_to_file, data_count):

    data = [[] for i in range(data_count)]

    with open(path_to_file, "r") as filestream:
        for line in filestream:
            currentline = line.split(",")
            
            for i, item in enumerate(currentline):
                data[i].append(float(item))

    return data



mse_default = read_from_csv_test("resources/logs/ekf_default_map_2/log_mse.txt", 1)[0]
mse_own = read_from_csv_test("resources/logs/ekf_own_map_all/log_mse.txt", 1)[0]

plt.figure()

plt.subplot(1,2,1)
plt.title("MSE - Default map")
plt.plot(mse_default[150:])
plt.grid("on")

plt.subplot(1,2,2)
plt.title("MSE - Own map")
plt.plot(mse_own[200:])
plt.grid("on")

plt.show()


odom_default = read_from_csv_test("resources/logs/ekf_default_map_2/log_odom.txt", 3)
odom_drift_default = read_from_csv_test("resources/logs/ekf_default_map_2/log_odom_drift.txt", 3)
odom_corr_default = read_from_csv_test("resources/logs/ekf_default_map_2/log_odom_corr.txt", 3)

odom_default_own = read_from_csv_test("resources/logs/ekf_own_map_all/log_odom.txt", 3)
odom_drift_default_own = read_from_csv_test("resources/logs/ekf_own_map_all/log_odom_drift.txt", 3)
odom_corr_default_own = read_from_csv_test("resources/logs/ekf_own_map_all/log_odom_corr.txt", 3)

plt.figure()

plt.subplot(1,2,1)
plt.title("Trajectory - Default map")
plt.plot(odom_default[0], odom_default[1], label="odometry")
plt.plot(odom_drift_default[0], odom_drift_default[1], label="noised odometry")
plt.plot(odom_corr_default[0], odom_corr_default[1], label="estimated odometry")
plt.grid("on")
plt.legend(loc="best")

plt.subplot(1,2,2)
plt.title("Trajectory - Own map")
plt.plot(odom_default_own[0], odom_default_own[1], label="odometry")
plt.plot(odom_drift_default_own[0], odom_drift_default_own[1], label="noised odometry")
plt.plot(odom_corr_default_own[0], odom_corr_default_own[1], label="estimated odometry")
plt.grid("on")
plt.legend(loc="best")

plt.show()