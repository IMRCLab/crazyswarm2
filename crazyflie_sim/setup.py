from setuptools import find_namespace_packages, setup

package_name = 'crazyflie_sim'

setup(
    name=package_name,
    version='1.0.7',
    # backend and visualization are plain directories (no __init__.py)
    packages=find_namespace_packages(
        include=[package_name, package_name + '.*'],
        exclude=['*.data', '*.data.*']),
    package_data={
        package_name: [
            'backend/data/*/*',
            'visualization/data/*/*',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wolfgang H\u00f6nig, Kimberly N. McGuire',
    maintainer_email='hoenig@tu-berlin.de, kimberleymcguire@gmail.com',
    description='Simulator for the Crazyswarm2 ROS stack',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'crazyflie_server = crazyflie_sim.crazyflie_server:main'
        ],
    },
)
