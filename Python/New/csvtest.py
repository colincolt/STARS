import csv
import random
import time

start = time.time()
while time.time()-start < 0.2:
    distance = float(random.choice([20.2,21.3,22.2,22.5,23.2,23.8,24.4]))
    myData = [{distance},]
    myFile = open('/home/treen/Desktop/csvexample3.csv', 'w')
    with myFile:
       writer = csv.writer(myFile)
       writer.writerow(myData)