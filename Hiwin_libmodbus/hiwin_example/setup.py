from setuptools import setup
import os
from glob import glob

package_name = 'hiwin_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andy',
    maintainer_email='808790017@gms.tku.edu.tw',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'strategy_example = hiwin_example.strategy_example:main',
            # 'hand_in_eye_calibration = hiwin_example.hand_in_eye_calibration:main',
            # 'three_points_calibration_example = hiwin_example.three_points_calibration_example:main',
            'hiwin_new=hiwin_example.hiwin_new:main',
            'read_object=hiwin_example.read_object:main',
            'read=hiwin_example.order_sub:main',
        ],
    },
)
