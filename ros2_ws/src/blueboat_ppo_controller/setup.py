from glob import glob
from setuptools import setup

setup(name='blueboat_ppo_controller', version='0.1.0',
    description='BlueBoat PPO speed and yaw-rate tracking with guarded MAVROS integration',
    license='LicenseRef-See-Provenance', packages=['blueboat_ppo_controller'],
    package_data={'blueboat_ppo_controller': ['assets/*', 'profiles/alberto/*']},
    data_files=[('share/ament_index/resource_index/packages', ['resource/blueboat_ppo_controller']),
        ('share/blueboat_ppo_controller', ['package.xml', 'PROVENANCE.md']),
        ('share/blueboat_ppo_controller/config', glob('config/*')),
        ('share/blueboat_ppo_controller/launch', glob('launch/*.py'))],
    install_requires=['setuptools', 'numpy>=1.24,<3', 'scipy>=1.10,<2', 'PyYAML>=6,<7', 'onnxruntime>=1.17,<2'],
    extras_require={'calibration': ['matplotlib>=3.6,<4'], 'handoff': ['pymavlink>=2.4.41,<3']},
    entry_points={'console_scripts': ['ppo_node = blueboat_ppo_controller.ppo_node:main', 'inference_test = blueboat_ppo_controller.inference_test:main',
        'alberto_boat = blueboat_ppo_controller.boat_profile:main',
        'blueboat_gateway = blueboat_ppo_controller.mavlink_gateway:main',
        'blueboat_owner = blueboat_ppo_controller.ground_owner:main',
        'blueboat_simple_mission = blueboat_ppo_controller.mission_node:main',
        'unchecked_calibration = blueboat_ppo_controller.unchecked_calibration:main',
        'thrust_calibration = blueboat_ppo_controller.calibration_node:main',
        'calibration_analyze = blueboat_ppo_controller.calibration_analysis:main',
        'calibration_synthetic = blueboat_ppo_controller.calibration_synthetic:main']})
