from setuptools import setup
import os
from glob import glob
package_name = 'eic_adi_sensor_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Einfochips pvt ltd.',
    maintainer_email='support@einfochips.com',
    description='Package contains launch file and configurations to run robot_localization package',
    license='Apache2.0, BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
