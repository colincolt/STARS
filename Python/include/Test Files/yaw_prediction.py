import csv
from collections import deque
import numpy as np

pixel_disps = deque([])
max_measures = 20

with open('/home/treen/Documents/School/5.Winter 2019/4A06 S.T.A.R.S/STARS_PROGRAM/STARS/Test Files/data/pixeltest1.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        data_point = str(row[0])
        data_point = data_point.strip(' ')
        data_point = int(data_point)

        new_pix = data_point

        if len(pixel_disps) == max_measures:
            mov_pix_avgs = np.convolve(pixel_disps, np.ones((5)) / 5, mode='valid')
            print("pixel disp: ", pixel_disps)
            if abs(new_pix - mov_pix_avgs[15]) <= 2000:  # << REALLY LARGE SO IT WONT REALLY AFFECT ANYTHING YET
                pix_replace = 0
                pixel_disps.append(new_pix)
            #                print("stereo ",distances)
            else:
                pix_replace += 1
                if pix_replace <= 10:
                    p_slope1 = mov_pix_avgs[15] - mov_pix_avgs[14]
                    p_slope2 = mov_pix_avgs[14] - mov_pix_avgs[13]
                    avg_pix_change = (p_slope1 + p_slope2) / 2
                    new_pix = float(round(pixel_disps[19] + avg_pix_change, 2))
                    # print(new_distance)
                    pixel_disps.append(new_pix)
                    print("replaced pixel disp: ", pixel_disps)
            pixel_disps.popleft()

            # if boost_yaw.is_set():  # THIS IS FOR THE DYNAMIC DRILL ONLY __________________________________________________________#

            if mov_pix_avgs[15] < 0:
                # Yaw is moving RIGHT?
                if abs(pixel_disps[19]) < 100 or pixel_disps[19] > 0:
                    new_pix = 500 + int(mov_pix_avgs[15])
                    print("Moving Left")
                else:
                    new_pix = - 1000 + int(mov_pix_avgs[15])
                    print("Moving Right")
                print("if Yaw_boost is set, will get sent: ", new_pix)
            else:
                # Yaw is moving LEFT?
                if abs(pixel_disps[19]) < 100 or pixel_disps[19] < 0:
                    new_pix = - 500 + int(mov_pix_avgs[15])
                    print("Moving Right")
                else:
                    new_pix = 1000 + int(mov_pix_avgs[15])
                    print("Moving Left")
                print("if Yaw_boost is set, will get sent: ", new_pix)
        else:
            pixel_disps.append(new_pix)
            print("populating pixel disp: ... ", pixel_disps)

        # print(data_point)
        line_count += 1
    print(f'Processed {line_count} lines.')