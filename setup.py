from setuptools import find_packages, setup

package_name = 'prompt_turtlesim'

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
    maintainer='robotics',
    maintainer_email='robotics@todo.todo',
    description='Controlling the Turtlesim in ROS2 using simple prompts to control the robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prompt_turtle = prompt_turtlesim.prompt_turtle:main',
        ],
    },
)

