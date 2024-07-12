from setuptools import find_packages, setup

package_name = 'auto_navigation'

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
    maintainer='lucia',
    maintainer_email='lucia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy2cost=auto_navigation.occupancyToCost:main',
            'astar=auto_navigation.aStar:main',
            'costmapAdjust=auto_navigation.costmapAdjust:main'
        ],
    },
)
