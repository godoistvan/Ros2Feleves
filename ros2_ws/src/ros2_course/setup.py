from setuptools import find_packages, setup

package_name = 'ros2_course'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/ros2_course']),
    ('share/ros2_course', ['package.xml']),
    ('share/ros2_course/launch', ['launch/launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='godoistvan',
    maintainer_email='godopisti@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = ros2_course.hello:main',
            'koch_snowflake=ros2_course.feleves:main'
        ],
    },
)
