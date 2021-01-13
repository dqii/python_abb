from abb import Robot
import numpy as np
from scipy.spatial.transform import Rotation as scipy_rotation

def quat_to_euler(q):
	return list(np.around(scipy_rotation.from_quat(q).as_euler('xyz', degrees=True), 3))

def euler_to_quat(e):
	return list(np.around(scipy_rotation.from_euler('xyz', e, degrees=True).as_quat(), 3))

class RelativeRobot(Robot):
	def __init__(self, 
				 ip          = '192.168.125.1', 
				 port_motion = 5000,
				 port_logger = 5001):
		super().__init__(ip, port_motion, port_logger)

	def relative_move(self, dx=0, dy=0, dz=0, drx=0, dry=0, drz=0):
		old_pose = self.get_cartesian()

		# calculate cartesian
		(x, y, z) = old_pose[0]
		c = [x + dx, y + dy, z + dz]

		# calculate quaternion
		r = scipy_rotation.from_quat(old_pose[1])
		dr = scipy_rotation.from_euler('xyz', [drx, dry, drz], degrees=True)
		q = list((r * dr).as_quat())

		pose = [c, q]
		self.set_cartesian(pose)

	def point_up(self):
		c = self.get_cartesian()[0]		
		self.set_cartesian([c, [1,0,0,0]]) 

	def point_down(self):
		c = self.get_cartesian()[0]		
		self.set_cartesian([c, [0,0,1,0]])

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
		self.set_cartesian(pose)

	@y.setter
	def y(self, val):
		(x, _, z), q = self.get_cartesian()
		pose = [[x, val, z], q]
		self.set_cartesian(pose)

	@z.setter
	def z(self, val):
		(x, y, _), q = self.get_cartesian()
		pose = [[x, y, val], q]
		self.set_cartesian(pose)

	@rx.setter
	def rx(self, val):
		c, q = self.get_cartesian()
		(_, ry, rz) = quat_to_euler(q)
		pose = [c, euler_to_quat([val, ry, rz])]
		self.set_cartesian(pose)

	@ry.setter
	def ry(self, val):
		c, q = self.get_cartesian()
		(rx, _, rz) = quat_to_euler(q)
		pose = [c, euler_to_quat([rx, val, rz])]
		self.set_cartesian(pose)

	@rz.setter
	def rz(self, val):
		c, q = self.get_cartesian()
		(rx, ry, _) = quat_to_euler(q)
		pose = [c, euler_to_quat([rx, ry, val])]
		self.set_cartesian(pose)