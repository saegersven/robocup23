# adds all ml images without label to csv file

from pandas import *
import os
import shutil
import csv

csv_data = read_csv("victims_01.csv")
img_names_with_victim = csv_data["image"].tolist()
img_names_without_victim = [] # img names that remain once you remove all names from csv file

for image in os.listdir("data"):
    if image in img_names_with_victim:
        pass
    else:
        img_names_without_victim.append(image)

# add img names without victim to csv file
# csv format:
# image xmin    ymin    xmax    ymax    label
# ...   0       0       0       0       no_victim

with open('victims_01b.csv', 'a') as f:
    writer = csv.writer(f)
    for i in range(0, len(img_names_without_victim)):
        a = '"' + img_names_without_victim[i] + '"'
        writer.writerow([a, 0, 0, 0, 0, 'no_victim'])
f.close()

