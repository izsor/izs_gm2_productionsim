import os
from glob import glob
from setuptools import setup

package_name = 'izs_gm2_productionsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='izsor',
    maintainer_email='TODO',
    description='TODO',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'workpiece_generator = izs_gm2_productionsim.workpiece_generator:main',
            'measurement_station = izs_gm2_productionsim.measurement_station:main',
            'quality_inspector = izs_gm2_productionsim.quality_inspector:main',
        ],
    },
)
