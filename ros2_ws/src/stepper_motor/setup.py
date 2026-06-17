from setuptools import find_packages, setup

package_name = 'stepper_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'sensor_msgs', 'gpiozero', 'rpi-hardware-pwm'],
    zip_safe=True,
    maintainer='dawid',
    maintainer_email='dawid.pietras.91@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stepper = stepper_motor.stepper:main'
        ],
    },
)
