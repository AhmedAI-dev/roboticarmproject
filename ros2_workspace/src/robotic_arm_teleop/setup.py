from setuptools import find_packages, setup

package_name = 'robotic_arm_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmed Wassef',
    maintainer_email='wassefahmed883@gmail.com',
    description='GUI Manual Control Dashboard publishing /joint_states.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'node_gui_teleop = robotic_arm_teleop.gui_teleop_node:main'
        ],
    },
)
