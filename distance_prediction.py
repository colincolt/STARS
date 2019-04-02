import csv, time, sys
from collections import deque
import numpy as np
import matplotlib.pyplot as plt


distances = np.array([])
max_measures = 20
x_values = np.array(range(0, 20))
# weights = np.array([4,4,4,4,3,3,3,3,2,2,2,2,1,1,1,1,1,1,1,1])
# poly_fit = np.poly1d(np.polyfit(x_values, distances, 3, full=False))
plot = plt.plot()
point_num = 20

plt.ion()
plt.show()
print(x_values)
print(x_values[-1])
with open('/home/treen/Documents/School/5.Winter 2019/4A06 S.T.A.R.S/STARS_PROGRAM/STARS/Test Files/data/csv_test1.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=' ')
    line_count = 0
    for row in csv_reader:
        try:
            # time.sleep(0.2)
            data_point = str(row[0])
            data_point = data_point.strip(' ')
            data_point = float(data_point)

            # print(row)
            new_distance = data_point
            if len(distances) == max_measures:
                moving_avgs = np.convolve(distances, np.ones((5)) / 5, mode='valid')

                if abs(new_distance - moving_avgs[15]) <= 2.0:
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
                # ============================================================================================================ #
                point_num += 1

                # x_new = (x_values[-20:])
                # print(len(x_values))
                # print(len(distances[0:-1]))

                poly_fit = np.poly1d(np.polyfit(x_values, distances[0:-1], 2, full=False))

                predicted_point = x_values[-1]+10

                future_distance = poly_fit(predicted_point)

                difference = future_distance - distances[-1]
                boost_scaling = 2.5*(distances[-1]/25)

                if abs(difference) > boost_scaling:
                    if difference > 0:
                        future_distance = distances[-1] + boost_scaling

                    else:
                        future_distance = distances[-1] - boost_scaling

                # if abs(distances[0]) - abs(distances[-1]) <=0.6:
                #     future_distance = new_distance

                plot = plt.plot(x_values, distances[0:-1], '.', x_values, poly_fit(x_values), '-', predicted_point, future_distance, '*')
                # print(poly_fit)

                x_values = np.append(x_values,point_num)

                # plt.show()
                plt.pause(.15)

                # future_distance = poly_fit(predicted_point)
                # ============================================================================================================ #
                # print("if future distance were requested: ",future_distance)
                # future_flag.clear()

                # DETECT RUNNiNG PLAYER:
                # if abs(moving_avgs[15] - moving_avgs[0]) > 1.6:
                #     # player_running.set()
                #     print("[Mainfile]: PLAYER IS RUNNING")
                    # player_speed = (moving_avgs[15] - moving_avgs[0])/sum(times[2:17])
                # else:
                #     player_running = False



                distances = distances[1:]
                x_values = x_values[1:]
            else:
                distances = np.append(distances,new_distance)
                print("populating",distances)

            # print(data_point)
            line_count += 1
            # print(line_count)
        except KeyboardInterrupt as key:
            print("Program terminated")
            sys.exit()

    print(f'Processed {line_count} lines.')