from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'izs_gm2_productionsim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roland Izsó',
    maintainer_email='izso.roland@gmail.com',
    description='ROS 2 manufacturing line simulation (line -> measure -> quality -> stats).',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line = izs_gm2_productionsim.line_node:main',
            'device = izs_gm2_productionsim.measure_node:main',
            'quality = izs_gm2_productionsim.quality_node:main',
            'stats = izs_gm2_productionsim.stats_node:main',
        ],
    },
)
