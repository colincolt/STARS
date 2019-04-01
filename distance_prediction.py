import csv, time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt


distances = np.array([])
max_measures = 20
x_values = np.array(range(0, 20))
# poly_fit = np.poly1d(np.polyfit(x_values, distances, 3, full=False))
plot = plt.plot()
point_num = 20

plt.ion()
plt.show()
print(x_values)
print(x_values[-1])
with open('/home/treen/Documents/School/5.Winter 2019/4A06 S.T.A.R.S/STARS_PROGRAM/STARS/Test Files/data/csvTest3.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        # time.sleep(0.2)
        data_point = str(row[0])
        data_point = data_point.strip(' ')
        data_point = float(data_point)

        # print(row)
        new_distance = data_point
        if len(distances) == max_measures:
            moving_avgs = np.convolve(distances, np.ones((5)) / 5, mode='valid')
            # ============================================================================================================ #
            point_num += 1
            x_values = np.append(x_values,point_num)

            x_new = (x_values[-20:])
            # print(len(distances))

            poly_fit = np.poly1d(np.polyfit(x_new, distances, 3, full=False))

            predicted_point = x_new[-1]+5

            future_distance = poly_fit(predicted_point)

            difference = abs(future_distance) - distances[-1]
            if difference > 2.5:
                if difference > 0:
                    future_distance = distances[-1] + 2.5

                else:
                    future_distance = distances[-1] - 2.5


            plot = plt.plot(x_new, distances, '.', x_new, poly_fit(x_new), '-', predicted_point, future_distance, '*')
            print(poly_fit)

            # plt.show()
            plt.pause(.1)

            future_distance = poly_fit(predicted_point)
            # ============================================================================================================ #
            # print("if future distance were requested: ",future_distance)
            # future_flag.clear()

            # DETECT RUNNiNG PLAYER:
            if abs(moving_avgs[15] - moving_avgs[0]) > 1.6:
                # player_running.set()
                print("[Mainfile]: PLAYER IS RUNNING")
                # player_speed = (moving_avgs[15] - moving_avgs[0])/sum(times[2:17])
            # else:
            #     player_running = False

            if abs(new_distance - moving_avgs[15]) <= 2.5:
                distances = np.append(distances,new_distance)
                replacements = 0
                # print("distance ",distances)
            else:
                print("replacing", new_distance)
                replacements += 1
                if replacements <= 10:
                    slope1 = moving_avgs[14] - moving_avgs[13]
                    slope2 = moving_avgs[15] - moving_avgs[14]
                    avg_change = (slope1 + slope2) / 2
                    new_distance = float(round(distances[19] + avg_change, 2))
                    # print(new_distance)
                    distances = np.append(distances,new_distance)
                    # print("replaced ",distances)

            distances = distances[1:]
            x_values = x_values[1:]
        else:
            distances = np.append(distances,new_distance)
            print("populating",distances)

        # print(data_point)
        line_count += 1
        # print(line_count)
    print(f'Processed {line_count} lines.')