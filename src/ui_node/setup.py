from setuptools import setup
package_name = 'ui_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[                           # ← these two install the ament index marker
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Keyboard UI for game brain',
    license='MIT',
    entry_points={
    'console_scripts': [
        'player_ui  = ui_node.player_ui:main',   
        'player_gui = ui_node.player_gui:main',  # NEW window UI
    ],
},


)
