from setuptools import setup

package_name = 'runs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'camera_node',
        'lidar_node',
        'motor_control_node',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='R.u.n.s',
    maintainer_email='eng23ra0065@gmail.com',
    description='ROS2 package for self-driving car',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = runs.camera_node:main',
            'lidar_node = runs.lidar_node:main',
            'motor_control_node = runs.motor_control_node:main',
        ],
    },
)
