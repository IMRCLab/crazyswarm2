from setuptools import setup

package_name = 'crazyflie_py'

setup(
    name=package_name,
    version='1.0.5',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wolfgang Hönig, Kimberly N. McGuire',
    maintainer_email='hoenig@tu-berlin.de, kimberleymcguire@gmail.com',
    description='Simple Python interface for Crazyswarm2',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
