from setuptools import find_packages, setup

package_name = 'ur5e_gripper_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yli336',
    maintainer_email='yli336@todo.todo',
    description='Simple open/close interface for the Robotiq gripper controller.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gripper_command_server = ur5e_gripper_control.gripper_command_server:main',
        ],
    },
)
