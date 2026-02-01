from setuptools import setup

package_name = 'pruebas_pablo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablomnt16',
    maintainer_email='pablomnt16@todo.todo',
    description='Markers for drone waypoints',
    license='TODO',
    entry_points={
        'console_scripts': [
            'markers = pruebas_pablo.markers:main',
        ],
    },
)
