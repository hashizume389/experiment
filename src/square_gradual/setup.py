from setuptools import setup, find_packages

package_name = 'square_gradual'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takayuki Hashizume',
    maintainer_email='AE1212@nara.kosen-ac.jp',
    description='Visual SLAMに基づいた正方形軌跡プログラム（徐行制御付き）',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'square_gradual = square_gradual.square_gradual:main',
        ],
    },
)
