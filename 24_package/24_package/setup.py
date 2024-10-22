from setuptools import find_packages, setup

package_name = '24_package'

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
    maintainer='vl11',
    maintainer_email='vl11@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "closed_loop_informed_Q_RRT_star = 24_package.closed_loop_controller_informed_Q_RRT_star:main",
            "closed_loop_Q_RRT_star = 24_package.closed_loop_controller_Q_RRT_star:main",
            "closed_loop_RRT_star = 24_package.closed_loop_controller_RRT_star:main",
            "closed_loop_informed_RRT_star = 24_package.closed_loop_controller_informed_RRT_star:main",
            "closed_loop_RRT = 24_package.closed_loop_controller_RRT:main"
        ],
    },
)
