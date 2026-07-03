from setuptools import find_packages, setup

package_name = 'controllers'

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
    maintainer='Giovanny Baño',
    maintainer_email='giovanny.bano@espol.edu.ec',
    description='Controlador reactivo Follow The Gap para F1TENTH (pista Brands Hatch)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap_node = controllers.gap_node:main',
        ],
    },
)
