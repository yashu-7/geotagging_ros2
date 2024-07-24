from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'geotagging'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edith',
    maintainer_email='edith@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'campub=geotaggin.cam_pub:main',
            'camsub=geotagging.cam_sub:main',
            'imgsave=geotagging.imagesave:main',
            'buff=geotagging.buffer:main',
            'imgbytes=geotagging.img_bytes:main',
            'trigger=geotagging.trigger:main',

        ],
    },
)
