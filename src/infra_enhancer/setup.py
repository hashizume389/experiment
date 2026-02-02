from setuptools import setup
import os
from glob import glob

package_name = 'infra_enhancer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install scripts to lib/<package_name> for ros2 run to find them
        ('lib/' + package_name, glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Hashizume Takayuki',
    author_email='AE1212@nara.kosen-ac.jp',
    description='Infrared image preprocessor',
    entry_points={
        'console_scripts': [
            'infra_enhancer_node = infra_enhancer.enhancer_node:main'
        ],
    },
)
