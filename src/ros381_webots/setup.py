from setuptools import find_packages, setup

package_name = 'ros381_webots'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hostuser',
    maintainer_email='l.popadic.2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = ros381_webots.motor_driver:main',
            'encoder_reader = ros381_webots.encoder_reader:main',
            'lidar_reader = ros381_webots.lidar_reader:main',
            'global_chinch = ros381_webots.global_chinch:main',
        ]
    },
)
