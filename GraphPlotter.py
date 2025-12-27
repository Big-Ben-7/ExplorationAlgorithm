import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import csv
#xEndpt, yEndpt, xNone, yNone, xOxygen, yOxygen, xFood, yFood, xHostile, yHostile, rHostile
data = []
with open("PlotterData.csv", mode='r', newline = '') as file:
    reader = csv.reader(file)
    for row in reader:
        rowData = [float(element) for element in row]
        data.append(rowData)
fig, ax = plt.subplots()
index = 0
plt.scatter(data[index][0], data[index+1][0], color='black', label='source')
plt.scatter(data[index][1], data[index+1][1], color='purple', label='sink')
index += 2
try:
    plt.scatter(data[index], data[index+1], color='purple', label='normal')
    index += 2
except:
    pass
try:
    plt.scatter(data[index], data[index+1], color='blue', label='oxygen')
    index += 2
except:
    pass
try:
    plt.scatter(data[index], data[index+1], color='green', label='food')
    index += 2
except:
    pass
try:
    plt.scatter(data[index], data[index+1], color='red', label='hostile')
    for i in range(len(data[index])):
        circle = Circle((data[index][i], data[index+1][i]), data[index+2][i], color='red', fill=False)
        ax.add_patch(circle)
    index += 3
except:
    pass
try:
    plt.plot(data[index], data[index+1], linestyle='-', color='orange')
    index += 2
except:
    pass
for i in range(index, len(data)-1, 2):
    plt.plot(data[i], data[i+1], linestyle='-', color='black')
ax.set_aspect('equal', 'box')
plt.title('Data')
plt.xlabel('x')
plt.ylabel('y')
plt.show()