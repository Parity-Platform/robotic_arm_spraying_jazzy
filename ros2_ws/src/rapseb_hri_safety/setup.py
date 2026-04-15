from setuptools import setup

package_name = 'rapseb_hri_safety'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/hri_safety_guard.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Parity Platform P.C.',
    maintainer_email='info@parityplatform.com',
    description='HRI proximity safety guard for the RAPSEB UR10e workcell.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hri_safety_guard = rapseb_hri_safety.hri_safety_guard:main',
        ],
    },
)
