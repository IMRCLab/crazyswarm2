from glob import glob

from setuptools import find_namespace_packages, setup

package_name = 'crazyflie_examples'

setup(
    name=package_name,
    version='1.0.7',
    packages=find_namespace_packages(
        include=[package_name, package_name + '.*'],
        exclude=['*.data', '*.data.*']),
    package_data={
        package_name: [
            'data/*.csv',
            'data/multi_trajectory/*.csv',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/data', glob('data/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wolfgang H\u00f6nig, Kimberly McGuire',
    maintainer_email='hoenig@tu-berlin.de, kimberleymcguire@gmail.com',
    description='Examples for the Crazyswarm2 ROS stack',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'hello_world = crazyflie_examples.hello_world:main',
            'nice_hover = crazyflie_examples.nice_hover:main',
            'figure8 = crazyflie_examples.figure8:main',
            'goto_unicast = crazyflie_examples.goto_unicast:main',
            'group_mask = crazyflie_examples.group_mask:main',
            'multi_trajectory = crazyflie_examples.multi_trajectory:main',
            'cmd_full_state = crazyflie_examples.cmd_full_state:main',
            'set_param = crazyflie_examples.set_param:main',
            'swap = crazyflie_examples.swap:main',
            'infinite_flight = crazyflie_examples.infinite_flight:main',
            'arming = crazyflie_examples.arming:main',
            'vel_mux = crazyflie_examples.vel_mux:main',
            'simple_mapper_multiranger = crazyflie_examples.simple_mapper_multiranger:main'
        ],
    },
)
