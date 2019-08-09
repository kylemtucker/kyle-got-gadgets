
import numpy, matplotlib, csv, math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np



def laserscan_to_pointcloud(csvline):
	time = csvline[0]
	angle = float(csvline[1])
	data = csvline[2:]

	points = laserscan_to_xyz(data, angle)

	data = []

	intensity = 

	index = 0
	for point in points:

		data.extend([index, intensity, point[2], point[1], point[0]])











		index += 1 





	retVal = PointCloud2()
	retVal.header = Header()

	retVal.height = 1
	retVal.width = len(dataArray)

def laserscan_to_xyz(laserscan, x_angular):
	points = []
	if len(laserscan) == 720:
		currAngle = -math.pi
		degInc = math.pi / 360
		for elem in laserscan:
			elem = float(elem)
			if elem > 0 and elem < 10:

				xVal = dist * math.cos(currAngle)
				yVal = dist * math.sin(currAngle)
				zVal = yVal * math.sin(x_angular)
				yVal = yVal * math.cos(x_angular)

				points.append([xVal, yVal, zVal])

				currAngle += degInc

			else:
				currAngle += degInc

	return points







def analyze_csv(csvFile):
	reader = csv.reader(csvFile)

	degInc = (2 * math.pi / 720)
	lidarRotAngle = -math.pi

	x_vals = []
	y_vals = []
	z_vals = []

	count = 0


	for line in reader:
		print("Printing a line!")
		if count == 3:
			print("Breaking foor loop")
			break

		count += 1

		currTime = line[0]
		currAngle = eval(line[1])[0]
		data = line[2:]

		for dist in data:
			dist = float(dist)
			
			if dist == 0.:
				lidarRotAngle += degInc

			else:
				yVal = dist * math.cos(lidarRotAngle)
				xVal = dist * math.sin(lidarRotAngle)
				zVal = yVal * math.sin(currAngle)
				yVal = dist * math.cos(currAngle)

				x_vals.append(xVal)
				y_vals.append(yVal)
				z_vals.append(zVal)

				lidarRotAngle += degInc

		lidarRotAngle = -math.pi


	print("settin up matplotlib")
	xs = x_vals
	ys = y_vals
	zs = z_vals


	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(xs, ys, zs, s=1)

	print("show plot")
	plt.show()







def main(std_args):
	# Pull std args from cmd line
	# Locate file
	# Locate analysis mode
	# Plot!

	csvFile = open("./log.csv")
	analyze_csv(csvFile)

main(0)

