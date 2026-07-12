from setuptools import find_packages, setup

package_name = 'vehicle_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Chassis serial driver for mc_base protocol',
    license='Apache-2.0',
    extras_require={},
    entry_points={
        'console_scripts': [
            'vehicle_driver = vehicle_driver.vehicle_driver_node:main',
            'keyboard_test = vehicle_driver.keyboard_test_node:main',
        ],
    },
)
