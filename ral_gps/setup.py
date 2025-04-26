from setuptools import find_packages, setup

package_name = 'ral_gps'

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
    maintainer='octa',
    maintainer_email='oeaguila@uc.cl',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wgs84_to_utm = ral_gps.wgs84_to_utm:main'
        ],
    },
)
