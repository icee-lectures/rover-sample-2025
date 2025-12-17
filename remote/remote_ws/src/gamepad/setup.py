from setuptools import find_packages, setup

package_name = 'gamepad'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gamepad.launch.py']),
    ],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='t0kage3',
    maintainer_email='t0kage3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gamepad_publisher=gamepad.gamepad_publisher:main',
            'joy_to_cmd_vel=gamepad.joy_to_cmd_vel:main',
        ],
    },
)
