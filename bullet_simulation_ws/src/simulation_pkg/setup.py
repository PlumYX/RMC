from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simulation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.*')),
        (os.path.join('share', package_name, 'meshes/collision'), glob('meshes/collision/*.*')),
        (os.path.join('share', package_name, 'meshes/visual'), glob('meshes/visual/*.*')),
        (os.path.join('share', package_name, 'meshes/dexh13_left/meshes'), glob('meshes/dexh13_left/meshes/*.*')),
        (os.path.join('share', package_name, 'meshes/dexh13_right/meshes'), glob('meshes/dexh13_right/meshes/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lyx',
    maintainer_email='1712306800@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_mimicry_simulation_node = simulation_pkg.hand_mimicry_simulation_node:main',
            'hand_angles_compute_node = simulation_pkg.hand_angles_compute_node:main'
        ],
    },
)
