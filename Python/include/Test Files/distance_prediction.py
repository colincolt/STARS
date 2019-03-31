import csv
from collections import deque
import numpy as np

distances = deque([])
max_measures = 20

with open('/home/treen/Documents/School/5.Winter 2019/4A06 S.T.A.R.S/STARS_PROGRAM/STARS/Test Files/data/csvTest3.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        data_point = str(row[0])
        data_point = data_point.strip(' ')
        data_point = float(data_point)

        new_distance = data_point
        if len(distances) == max_measures:
            moving_avgs = np.convolve(distances, np.ones((5)) / 5, mode='valid')

            # if future_flag.is_set():
            looped = 0
            while looped < 15:
                slope1 = moving_avgs[14] - moving_avgs[13]
                slope2 = moving_avgs[15] - moving_avgs[14]
                avg_change = (slope1 + slope2) / 2
                fut_distance = float(round(distances[19] + avg_change*4, 2))
                # print(new_distance)
                # distances.append(new_distance)
                looped += 1
            future_distance = fut_distance
            print("if future distance were requested: ",future_distance)
            # future_flag.clear()

            # DETECT RUNNiNG PLAYER:
            if abs(moving_avgs[15] - moving_avgs[0]) > 1.5:
                # player_running.set()
                print("[Mainfile]: PLAYER IS RUNNING")
                # player_speed = (moving_avgs[15] - moving_avgs[0])/sum(times[2:17])
            # else:
            #     player_running = False

            if abs(new_distance - moving_avgs[15]) <= 2.5:
                distances.append(new_distance)
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
                    distances.append(new_distance)
                    print("replaced ",distances)

            distances.popleft()
        else:
            distances.append(new_distance)
            print("populating",distances)

        # print(data_point)
        line_count += 1
    print(f'Processed {line_count} lines.')