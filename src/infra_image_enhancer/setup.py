from setuptools import setup

package_name = 'infra_image_enhancer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hassh',
    maintainer_email='hassh@example.com',
    description='RealSense赤外線画像の前処理ノード（CLAHE, ノイズ除去, 正規化, 同期）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'infra_image_enhancer = infra_image_enhancer.enhancer_node:main',
        ],
    },
)
