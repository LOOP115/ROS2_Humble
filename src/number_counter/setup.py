from setuptools import find_packages, setup

package_name = 'number_counter'

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
    maintainer='loop',
    maintainer_email='54584749+LOOP115@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "number_publisher = number_counter.number_publisher:main",
            "number_counter = number_counter.number_counter:main",
            "number_subscriber = number_counter.number_subscriber:main",
            "reset_number_counter = number_counter.reset_number_counter:main"
        ],
    },
)
