from setuptools import setup, find_packages

package_name = 'rotate_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takayuki Hashizume',
    maintainer_email='AE1212@nara.kosen-ac.jp',
    description='ホイールオドメトリに基づいた旋回プログラム',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rotate_odom = rotate_odom.rotate_odom:main',
        ],
    },
)
