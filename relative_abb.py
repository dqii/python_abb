from .abb import Robot
import numpy as np
from scipy.spatial.transform import Rotation
import time

def quat_to_euler(q):
	return list(np.around(Rotation.from_quat(q).as_euler('zyx', degrees=True), 3))

def euler_to_quat(e):
	return list(np.around(Rotation.from_euler('zyx', e, degrees=True).as_quat(), 3))

def euclidean_distance(p1, p2):
	return np.linalg.norm(np.array(p1) - np.array(p2))

def close_enough(p1, p2, error):
	xyz_error = euclidean_distance(p1[0], p2[0])
	return xyz_error < error

class RelativeRobot(Robot):
	def __init__(self, 
				 ip          = '192.168.125.1', 
				 port_motion = 5000,
				 port_logger = 5001):
		super().__init__(ip, port_motion, port_logger)
		print("Connected to ABB robot")

	def wait(self, goal):
		last = self.get_cartesian()
		t = time.time()
		while True:
			current = self.get_cartesian()
			if close_enough(current, goal, 0.05):
				break
			elif time.time() - t > 2:
				if close_enough(current, goal, 0.1) and close_enough(current, last, 0.01):
					break
				print("Goal", goal)
				print("Current", current)
				print("")
			last = current

	def get_euler(self):
		return quat_to_euler(self.get_cartesian()[1])

	def set_pose(self, pose, wait):
		self.movej(pose)
		if wait:
			self.wait(pose)

	def validate_inputs(self, x, y, z, rx, ry, rz, dx, dy, dz, drx, dry, drz):
		return (
			(x is None      or dx == 0)
			and (y is None  or dy == 0)
			and (z is None  or dz == 0)
			and (rx is None or drx == 0)
			and (ry is None or dry == 0)
			and (rz is None or drz == 0)
		)

	def move(self, x=None, y=None, z=None, rx=None, ry=None, rz=None, dx=0, dy=0, dz=0, drx=0, dry=0, drz=0, wait=True, linear=True):
		if not self.validate_inputs(x, y, z, rx, ry, rz, dx, dy, dz, drx, dry, drz):
			return

		old_pose = self.get_cartesian()

		# calculate cartesian
		(old_x, old_y, old_z) = old_pose[0]
		new_x = x if x is not None else old_x + dx
		new_y = y if y is not None else old_y + dy
		new_z = z if z is not None else old_z + dz
		c = [new_x, new_y, new_z]

		# calculate quaternion
		r = Rotation.from_quat(old_pose[1])
		if rx is not None or ry is not None or rz is not None:
			(old_rx, old_ry, old_rz) = r.as_euler('zyx', degrees=True)
			new_rx = rx if rx is not None else old_rx
			new_ry = ry if ry is not None else old_ry
			new_rz = rz if rz is not None else old_rz
			r = Rotation.from_euler('zyx', [new_rx, new_ry, new_rz], degrees=True)
		dr = Rotation.from_euler('zyx', [drx, dry, drz], degrees=True)
		q = list((r * dr).as_quat())

		# move to pose
		pose = [c, q]
		if linear:
			self.movel(pose)
		else:
			self.movej(pose)
		if wait:
			self.wait(pose)

	def point_down(self, wait=True):
		self.move(rx=180, ry=0, rz=0)

	def point_up(self, wait=True):
		self.move(rx=0, ry=0, rz=0)

	@property
	def x(self):
		return self.get_cartesian()[0][0]

	@property
	def y(self):
		return self.get_cartesian()[0][1]

	@property
	def z(self):
		return self.get_cartesian()[0][2]

	@property
	def rx(self):
		q = self.get_cartesian()[1]
		return quat_to_euler(q)[0]

	@property
	def ry(self):
		q = self.get_cartesian()[1]
		return quat_to_euler(q)[1]

	@property
	def rz(self):
		q = self.get_cartesian()[1]
		return quat_to_euler(q)[2]

	@x.setter
	def x(self, val):
		(_, y, z), q = self.get_cartesian()
		pose = [[val, y, z], q]
		self.movej(pose)

	@y.setter
	def y(self, val):
		(x, _, z), q = self.get_cartesian()
		pose = [[x, val, z], q]
		self.movej(pose)

	@z.setter
	def z(self, val):
		(x, y, _), q = self.get_cartesian()
		pose = [[x, y, val], q]
		self.movej(pose)

	@rx.setter
	def rx(self, val):
		c, q = self.get_cartesian()
		(_, ry, rz) = quat_to_euler(q)
		pose = [c, euler_to_quat([val, ry, rz])]
		self.movej(pose)

	@ry.setter
	def ry(self, val):
		c, q = self.get_cartesian()
		(rx, _, rz) = quat_to_euler(q)
		pose = [c, euler_to_quat([rx, val, rz])]
		self.movej(pose)

	@rz.setter
	def rz(self, val):
		c, q = self.get_cartesian()
		(rx, ry, _) = quat_to_euler(q)
		pose = [c, euler_to_quat([rx, ry, val])]
		self.movej(pose)