from setuptools import find_packages, setup

package_name = 'helpers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chirag',
    maintainer_email='chirag@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "cmd_relay = helpers.relay:main",
            "map_validator = helpers.map_validator:main",
            "data_logger = helpers.data_logger:main",
            "reactive_explorer = helpers.reactive_explorer:main",
        ],
    },
)
