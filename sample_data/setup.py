from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sample_data'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/config', ['config/arms.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Feiyang Wu',
    maintainer_email='feiyangw.21@intl.zju.edu.cn',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample = sample_data.sample:main'
        ],
    },
)
