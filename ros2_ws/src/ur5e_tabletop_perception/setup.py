from setuptools import find_packages, setup

package_name = 'ur5e_tabletop_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yli336',
    maintainer_email='yli336@todo.todo',
    description='Tabletop perception and supervisor for UR5e pick-place stack',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tabletop_perception_node = ur5e_tabletop_perception.tabletop_perception_node:main',
            'supervisor_node = ur5e_tabletop_perception.supervisor_node:main',
        ],
    },
)
