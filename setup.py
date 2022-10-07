from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bass',
    maintainer_email='abdelrhman_bassuny@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_sub = py_pubsub.odom_sub:main',
            'scan_sub = py_pubsub.scan_sub:main',
            'vel_pub = py_pubsub.vel_pub:main',
            'obstacle_stopper = py_pubsub.obstacle_stopper:main'
        ],
    },
)
