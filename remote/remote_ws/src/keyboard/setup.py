from setuptools import find_packages, setup

package_name = 'keyboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wasd_controller.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tkage',
    maintainer_email='tkageyama@tottori-u.ac.jp',
    description='キーボード入力でロボットを制御するパッケージ',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_publisher = keyboard.keyboard_publisher:main',
            'wasd_controller = keyboard.wasd_controller:main',
        ],
    },
)
