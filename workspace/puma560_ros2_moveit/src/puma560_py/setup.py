from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puma560_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cadlab',
    maintainer_email='cadlab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_space_motion = puma560_py.joint_space_motion:main',
            'cartesian_motion = puma560_py.cartesian_motion:main',
            'compliance_control = puma560_py.compliance_control:main',
            'constrained_motion = puma560_py.constrained_motion:main',
        ],
    },
)
