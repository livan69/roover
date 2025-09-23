from setuptools import find_packages, setup

package_name = 'roover_imu'

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
    maintainer='livan',
    maintainer_email='livanwils@todo.todo',
    description='IMU node for Roover (LSM6DSOX + LIS3MDL)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_node = roover_imu.imu_node:main',
        ],
    },
)
