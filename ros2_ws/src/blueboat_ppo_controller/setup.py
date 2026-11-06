from glob import glob
from setuptools import setup

setup(name='blueboat_ppo_controller', version='0.1.0',
    description='BlueBoat PPO speed and yaw-rate tracking with guarded MAVROS integration',
    license='LicenseRef-See-Provenance', packages=['blueboat_ppo_controller'],
    package_data={'blueboat_ppo_controller': ['assets/*']},
    data_files=[('share/ament_index/resource_index/packages', ['resource/blueboat_ppo_controller']),
        ('share/blueboat_ppo_controller', ['package.xml', 'PROVENANCE.md']),
        ('share/blueboat_ppo_controller/config', glob('config/*')),
        ('share/blueboat_ppo_controller/launch', glob('launch/*.py'))],
    install_requires=['setuptools', 'numpy>=1.24,<3', 'scipy>=1.10,<2', 'onnxruntime>=1.17,<2'],
    entry_points={'console_scripts': ['ppo_node = blueboat_ppo_controller.ppo_node:main', 'inference_test = blueboat_ppo_controller.inference_test:main']})
