from setuptools import setup

package_name = 'infra_enhancer'

setup(
    name=package_name,
    version='0.0.1',
    packages=["infra_enhancer"],
    data_files=[
        ('share/' + package_name, ['package.xml']),
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
