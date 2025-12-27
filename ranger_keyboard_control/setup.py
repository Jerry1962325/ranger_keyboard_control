from setuptools import setup

package_name = 'ranger_keyboard_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Keyboard control for Ranger Mini V3',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = ranger_keyboard_control.keyboard_control_node:main',
        ],
    },
)
