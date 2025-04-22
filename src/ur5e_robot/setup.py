from setuptools import find_packages, setup

package_name = 'ur5e_robot'

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
    maintainer='youssef',
    maintainer_email='youssef@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = ur5e_robot.camera_node : main",
            "test_node2 = ur5e_robot.camera_sub : main",
            "test_node3 = ur5e_robot.move_publisher : main",
            "test_node4 = ur5e_robot.move_sub : main",
            "test_node5 = ur5e_robot.robot_move: main"
        ],
    },
)
