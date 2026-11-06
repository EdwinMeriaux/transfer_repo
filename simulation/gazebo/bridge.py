from pathlib import Path
import time

import gz.math7
from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.sim8 import Link, Model, TestFixture, World, world_entity
from gz.transport13 import Node
import numpy as np
from scipy.spatial.transform import Rotation


class GazeboBoat:
    def __init__(self, world, effectiveness=1.0):
        self.world = Path(world).resolve()
        self.effectiveness = effectiveness
        self.link = None
        self.state = None
        self.odometry_count = 0
        self.last_thrust = np.zeros(2)
        self.node = Node()
        self.fixture = TestFixture(str(self.world))
        self.fixture.on_pre_update(self.pre_update)
        self.fixture.on_post_update(self.post_update)
        self.fixture.finalize()
        self.server = self.fixture.server()
        if not self.server.run(True, 2, False) or self.state is None:
            raise RuntimeError('Gazebo did not produce vessel state')
        self.topics = ['/model/blueboat/joint/' + name + '/cmd_thrust' for name in ('motor_port_joint', 'motor_stbd_joint')]
        self.publishers = [self.node.advertise(topic, Double) for topic in self.topics]
        self.node.subscribe(Odometry, '/model/blueboat/odometry', self.receive_odometry)
        deadline = time.monotonic() + 5.
        while not all(publisher.has_connections() for publisher in self.publishers):
            if time.monotonic() > deadline:
                raise RuntimeError('Gazebo thruster subscriptions were not discovered')
            time.sleep(.01)


    def pre_update(self, info, ecm):
        if self.link is None:
            world = World(world_entity(ecm))
            model = Model(world.model_by_name(ecm, 'blueboat'))
            self.link = Link(model.link_by_name(ecm, 'base_link'))
            self.link.enable_velocity_checks(ecm, True)


    def post_update(self, info, ecm):
        pose = self.link.world_pose(ecm)
        linear = self.link.world_linear_velocity(ecm)
        angular = self.link.world_angular_velocity(ecm)
        if pose is None or linear is None or angular is None:
            return
        position = np.array([pose.pos().x(), pose.pos().y(), pose.pos().z()])
        quaternion = np.array([pose.rot().x(), pose.rot().y(), pose.rot().z(), pose.rot().w()])
        velocity = np.array([linear.x(), linear.y(), linear.z()])
        omega = np.array([angular.x(), angular.y(), angular.z()])
        rotation = Rotation.from_quat(quaternion)
        self.state = dict(time_s=info.sim_time.total_seconds(), position=position, quaternion_xyzw=quaternion, velocity_world=velocity, omega_world=omega, body_velocity=rotation.inv().apply(velocity), body_omega=rotation.inv().apply(omega), euler_rpy=rotation.as_euler('xyz'))


    def receive_odometry(self, message):
        self.odometry_count += 1


    def step(self, thrust, physics_steps=4):
        thrust = np.asarray(thrust, dtype=float)
        if thrust.shape != (2,) or not np.isfinite(thrust).all():
            raise ValueError('Expected two finite port/starboard forces')
        self.last_thrust = np.clip(thrust, -15., 25.) * self.effectiveness
        for publisher, force in zip(self.publishers, self.last_thrust):
            if not publisher.publish(Double(data=float(force))):
                raise RuntimeError('Gazebo thrust publication failed')
        previous_time = self.state['time_s']
        if not self.server.run(True, physics_steps, False):
            raise RuntimeError('Gazebo stopped advancing')
        if not np.isclose(self.state['time_s'] - previous_time, physics_steps * .005, atol=1e-8):
            raise RuntimeError('Unexpected simulation timestep')
        if not all(np.isfinite(value).all() for value in self.state.values()):
            raise RuntimeError('Nonfinite Gazebo state')
        return self.state
