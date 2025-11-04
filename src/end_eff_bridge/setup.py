from setuptools import setup
package_name = 'end_eff_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/prongs_serial.launch.py']),
    ],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Simple serial bridge for end effector (Arduino)',
    license='MIT',
    entry_points={'console_scripts': ['bridge = end_eff_bridge.serial_bridge:main']},
)
