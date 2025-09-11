import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sprint_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Alvarez Vanegas',
    maintainer_email='da.alvarezv@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'unzipper = sprint_1.unzipper:main'
         ],
    },
    scripts=['scripts/yolo2graph_wrapper',
             'scripts/ontology_wrapper',],
)
