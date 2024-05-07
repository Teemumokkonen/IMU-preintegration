from setuptools import find_packages, setup

package_name = 'imu_preintegration'

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
    maintainer='localadmin',
    maintainer_email='teemu.mokkonen@tuni.fi',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pre_integration_node = imu_preintegration.imu_preintegration:main"
        ],
    },
)
