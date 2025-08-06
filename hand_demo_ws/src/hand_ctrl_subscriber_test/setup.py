from setuptools import find_packages, setup

package_name = 'hand_ctrl_subscriber_test'

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
    maintainer='lyx',
    maintainer_email='1712306800@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'hand_ctrl_subscriber_node = hand_ctrl_subscriber_test.hand_ctrl_subscriber_node:main',
        ],
    },
)
