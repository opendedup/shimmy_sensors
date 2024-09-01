from setuptools import find_packages, setup

package_name = 'shimmy_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sam Silverberg',
    maintainer_email='sam.silverberg@gmail.com',
    description='Sensor package for ros2 shimmy',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu = shimmy_sensors.ism330dhcx:main',
        ],
    },
)
