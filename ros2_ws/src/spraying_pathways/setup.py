from setuptools import setup

package_name = 'spraying_pathways'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Parity Platform P.C.',
    maintainer_email='info@parityplatform.com',
    description='Trajectory logger for robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trajectory_logger = scripts.trajectory_logger:main',
            'lidar_surface_scanner = scripts.lidar_surface_scanner:main',
        ],
    },
)