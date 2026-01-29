from setuptools import setup

package_name = 'agv_dummy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wein',
    maintainer_email='farhantamsa147@gmail.com',
    description='Dummy publishers for AGV (odom/tf) for development without hardware.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_dummy = agv_dummy.odom_dummy:main',
        ],
    },
)
