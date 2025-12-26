import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import csv
#xEndpt, yEndpt, xNone, yNone, xOxygen, yOxygen, xFood, yFood, xHostile, yHostile, rHostile
data = []
with open("PlotData.csv", mode='r', newline = '') as file:
    reader = csv.reader(file)
    for row in reader:
        rowData = [float(element) for element in row]
        data.append(rowData)
fig, ax = plt.subplots()
plt.scatter(data[0], data[1], color='orange', label='endpts')
plt.scatter(data[2], data[3], color='purple', label='normal')
plt.scatter(data[4], data[5], color='blue', label='oxygen')
plt.scatter(data[6], data[7], color='green', label='food')
plt.scatter(data[8], data[9], color='red', label='hostile')
for i in range(len(data[10])):
    circle = Circle((data[8][i], data[9][i]), data[10][i], color='red', fill=False)
    ax.add_patch(circle)
testIndex = 13
try:
    plt.plot(data[11], data[12], linestyle='-', color='orange')
except:
    testIndex = 11
for i in range(testIndex, len(data)-1, 2):
    plt.plot(data[i], data[i+1], linestyle='-', color='black')
ax.set_aspect('equal', 'box')
plt.title('Data')
plt.xlabel('x')
plt.ylabel('y')
plt.legend(loc='upper left')
plt.show()