from setuptools import setup

package_name = 'name_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/inverse_kinematics_controller.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Description of your package',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse_kinematics_controller = name_description.inverse_kinematics_controller:main',
        ],
    },
)


