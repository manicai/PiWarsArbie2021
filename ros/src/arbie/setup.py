from setuptools import setup

package_name = 'arbie'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyudev',
    ],
    zip_safe=True,
    maintainer='Ian Glover',
    maintainer_email='ian.glover@gmail.com',
    description='PiWars Arbie control code',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gamepad = arbie.gamepad_reader:main',
            'motor = arbie.motor_control:main',
        ],
    },
    classifiers=[
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
    ],
)
