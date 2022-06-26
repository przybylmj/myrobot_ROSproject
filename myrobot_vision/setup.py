from setuptools import setup
import os
from glob import glob

package_name = 'myrobot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'dnn'),glob('dnn/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jan',
    maintainer_email='przybyl@mj.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'myrobot_vision = myrobot_vision.myrobot_vision_proc:main',
        ],
    },
)
