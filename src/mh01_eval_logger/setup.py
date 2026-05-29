from glob import glob
from setuptools import find_packages, setup

package_name = 'mh01_eval_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takayuki Hashizume',
    maintainer_email='AE1212@nara.kosen-ac.jp',
    description='MH_01 rosbag evaluation logger for Visual SLAM odometry and ground truth.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mh01_eval_logger = mh01_eval_logger.mh01_eval_logger:main',
            'mh01_metrics = mh01_eval_logger.mh01_metrics_node:main',
        ],
    },
)
