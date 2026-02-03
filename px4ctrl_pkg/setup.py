from setuptools import find_packages, setup

package_name = 'px4ctrl_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/quad_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ugv-ume',
    maintainer_email='ugv-ume@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = px4ctrl_pkg.py_node:main",
            "quad_py_node = px4ctrl_pkg.quad_py_node:main",
            "quad_velocity_py_node = px4ctrl_pkg.quad_velocity_py_node:main"
        ],
    },
)
