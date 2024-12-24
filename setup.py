from setuptools import find_packages, setup

package_name = 'PID_CMD'

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
    maintainer='nyanziba',
    maintainer_email='nyanziba@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PID_CMD = PID_CMD.PID_CMD:main',
            'PID_CMD_SIM = PID_CMD.PID_CMD_turtlesim:main'
        ],
    },
)