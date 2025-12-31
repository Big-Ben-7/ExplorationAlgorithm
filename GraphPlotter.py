import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import csv
plt.ioff()
#xEndpt, yEndpt, xNone, yNone, xOxygen, yOxygen, xFood, yFood, xHostile, yHostile, rHostile, xPaths, yPaths
data = []
with open("PlotterData.csv", mode='r', newline = '') as file:
    reader = csv.reader(file)
    for row in reader:
        try:
            rowData = [float(element) for element in row]
            data.append(rowData)
        except:
            data.append("")
fig, ax = plt.subplots()
index = 0
plt.scatter(data[index][0], data[index+1][0], color='cyan', label='Earth')
plt.scatter(data[index][1], data[index+1][1], color='magenta', label='Target')
index += 2
if data[index] != "":
    plt.scatter(data[index], data[index+1], color='purple', label='Normal')
index += 2
if data[index] != "":
    plt.scatter(data[index], data[index+1], color='blue', label='Oxygen')
index += 2
if data[index] != "":
    plt.scatter(data[index], data[index+1], color='green', label='Food')
index += 2
if data[index] != "":
    plt.scatter(data[index], data[index+1], color='red', marker='.', s=1, label='Hostile')
    for i in range(len(data[index])):
        circle = Circle((data[index][i], data[index+1][i]), data[index+2][i], color='red', fill=False)
        ax.add_patch(circle)
index += 3
if data[index] != "":
    if len(data[index+1]) == len(data[index]):
        plt.plot(data[index], data[index+1], linestyle='-', marker='.', color='orange')
    else:
        plt.plot(data[index], data[index+1][:-1], linestyle='--', marker='.', color='orange') #Dashed = invalid path
        
index += 2
for i in range(index, len(data)-1, 2):
    plt.plot(data[i], data[i+1], linestyle='-', marker='.', color='black')
ax.set_aspect('equal', 'box')
plt.title('Data')
plt.xlabel('x')
plt.ylabel('y')
plt.legend
plt.show()
