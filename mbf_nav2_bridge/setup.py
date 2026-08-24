from setuptools import find_packages, setup

package_name = 'mbf_nav2_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Bridge mbf_msgs/action/MoveBase (/move_base) to nav2_msgs/action/NavigateToPose (/navigate_to_pose)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mbf_nav2_bridge = mbf_nav2_bridge.bridge:main',
        ],
    },
)
