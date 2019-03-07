import numpy as np
from collections import deque
import time

max_measures = 5
first_new = False
right_xcoords = deque([1236, 1240, 1256, 1300, 1260])
new_right_xcoord = 1600
start_time = time.time()

if len(right_xcoords) == max_measures:
    moving_avgs = np.convolve(right_xcoords, np.ones((3,))/3, mode='valid')
    if abs(new_right_xcoord - moving_avgs[2])<= 100:
        right_xcoords.append(new_right_xcoord)
        old_right_xcoord = new_right_xcoord
        first_new = True
    else:
        if first_new:
            right_xcoords.append(new_right_xcoord)
        else:
            slope1 = moving_avgs[1]-moving_avgs[0]
            slope2 = moving_avgs[2] - moving_avgs[1]
            avg_change = (slope1+slope1)/2
            new_right_xcoord = right_xcoords[4] + avg_change
            right_xcoords.append(new_right_xcoord)
            right_xcoords.popleft()
            print()



else:
    right_xcoords.append(new_right_xcoord)
    right_xcoords.popleft()
loop_time = time.time() - start_time
print("LOOP TIME:  ",loop_time)
print(right_xcoords)

