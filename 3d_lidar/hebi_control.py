import hebi, time, math

class LaserScanAndAngle:

	def __init__(self, laserScan, angle):

		self.laserScan = laserScan
		self.angle = angle

	def __repr__(self):
		return "LaserScan of length " + str(len(self.laserScan)) + " recorded at angle " + str(self.angle) + " rads."

class HebiForLidar:

	def __init__(self, actuation_interval, maxAngle, minAngle, bias, mac):

		self.lookup = hebi.Lookup()
		time.sleep(2.)
		self.group = self.lookup.get_group_from_macs([mac])
		self.group.feedback_frequency = 100
		self.cmd = hebi.GroupCommand(1)

		self.interval = actuation_interval

		self.angleBias = bias
		self.currAngle = 0
		self.maxAngle = self.angleBias + maxAngle
		self.minAngle = self.angleBias + minAngle

		self.degInc = 2 * (maxAngle - minAngle) / (self.interval * 7.)

		self.positiveAngular = True
		self.fixed = False
		self.angular_vel = math.pi

		self.killswitch = False
		self.oscillating = False

		self.lastTime = 0.

		self.init_hebi()


	def init_hebi(self):

		self.cmd.led.color = "green"

		self.cmd.position = self.angleBias

		timeStart = time.time()
		while time.time() - timeStart < 1:
			self.group.send_command(self.cmd)

		self.cmd.position = None


		self.group.add_feedback_handler(self.hebi_fbk_discrete)


		print("Lidar angle reset!")



		self.run_hebi()

	def run_hebi(self):
		time_count = 0
		while not self.killswitch and time_count < 1:
			time.sleep(1)
			time_count += 1

		return


	def hebi_fbk_continuous(self, group_fbk):

		self.cmd.led.color = "green"

		self.currAngle = group_fbk.position

		if self.currAngle > self.maxAngle + .1 or self.currAngle < self.minAngle - .1:
			self.killswitch =  True
			return

		if self.positiveAngular:
			if group_fbk.position >= self.maxAngle:
				self.positiveAngular = False
				self.cmd.velocity = -self.angular_vel
			else:
				self.cmd.velocity = self.angular_vel
			self.group.send_command(self.cmd)

		elif self.positiveAngular == False:
			if group_fbk.position < self.minAngle:
				self.positiveAngular = True
				self.cmd.velocity = self.angular_vel
			else:
				self.cmd.velocity = -self.angular_vel
			self.group.send_command(self.cmd)


	def hebi_fbk_discrete(self, group_fbk):
		# Half hertz means one scan in 2 seconds. Means rotate at pi/2 rads/s
		# 


		self.currAngle = group_fbk.position
		targetAngle = 0

		

		if self.currAngle >= self.maxAngle and self.positiveAngular:
			targetAngle = self.currAngle - self.degInc
			self.positiveAngular = False
			print("if 2")

		elif self.positiveAngular:
			targetAngle = self.currAngle + self.degInc
			print("if 3")

		elif self.currAngle <= self.minAngle and not self.positiveAngular:
			targetAngle = self.currAngle + self.degInc
			self.positiveAngular = True
			print("if 4")

		elif not self.positiveAngular:
			targetAngle = self.currAngle - self.degInc
			print("if 5")

		else:
			print("if 6")
			return

		
		if time.time() - self.lastTime < .18:
			print("keep angle")
			self.cmd.position = self.currAngle
		else:
			print("new angle")
			self.lastTime = time.time()
			self.cmd.position = targetAngle
			self.group.send_command(self.cmd)



	def get_curr_angle(self):
		return self.currAngle

