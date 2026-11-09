from setuptools import find_packages, setup

package_name = 'drone_aruco_task'

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
    maintainer='len',
    maintainer_email='len@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_pose_detector = drone_aruco_task.aruco_pose_detector:main',
            'drone_controller = drone_aruco_task.real_drone_controller:main',
            'sim_drone_controller = drone_aruco_task.sim_drone_controller:main',
        ],
    },
)
