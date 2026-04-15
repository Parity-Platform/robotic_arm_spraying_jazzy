from setuptools import setup

package_name = 'rapseb_teach_by_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/teach_demo.launch.py',
            'launch/extract_trajectory.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/logitech_f710.yaml',
            'config/servo_params.yaml',
        ]),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Parity Platform P.C.',
    maintainer_email='info@parityplatform.com',
    description='Teach-by-demonstration pipeline for the RAPSEB spraying workcell.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joy_teleop = rapseb_teach_by_demo.joy_teleop:main',
            'freedrive_gateway = rapseb_teach_by_demo.freedrive_gateway:main',
            'demo_recorder = rapseb_teach_by_demo.demo_recorder:main',
            'trajectory_extractor = rapseb_teach_by_demo.trajectory_extractor:main',
            'replay_publisher = rapseb_teach_by_demo.replay_publisher:main',
        ],
    },
)
