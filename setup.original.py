from setuptools import find_packages, setup

package_name = 'handsFree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/urdf', ['urdf/robot.xacro']),
        ('share/' + package_name + '/config', ['config/joystick.yaml', 'config/twist_mux.yaml', 'config/nav2_params.yaml', 'config/mapper_params_online_async.yaml']),
        ('share/' + package_name + '/launch', ['launch/init.launch.py', 'launch/control.launch.py', 'launch/nav.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sydney',
    maintainer_email='sydney@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = handsFree.MotionController:main',
            'state_publisher = handsFree.StatePublisher:main',
        ],
    },
)
