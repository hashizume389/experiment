from setuptools import setup
import os
from glob import glob

package_name = 'infra_enhancer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            # オールインワンノード（後方互換）
            'infra_enhancer_node = infra_enhancer.enhancer_node:main',
            # 分割ノード
            'denoise_node   = infra_enhancer.denoise_node:main',
            'clahe_node     = infra_enhancer.clahe_node:main',
            'normalize_node = infra_enhancer.normalize_node:main',
            # LDFE-SLAM Node
            'ldfe_node      = infra_enhancer.ldfe_node:main',
            # Spike Filter Node
            'spike_node     = infra_enhancer.spike_node:main',
            # Edge Detection Node
            'edge_node      = infra_enhancer.edge_node:main',
        ],
    },
)
