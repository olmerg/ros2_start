from setuptools import setup

package_name = 'py_basic_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olmerg',
    maintainer_email='olmerg@gmail.com',
    keywords=['ROS'],
    description='Exemplos para programar topicos no ros',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'talker = py_basic_examples.min_publisher:main',
             'listener = py_basic_examples.min_subscriber:main',
             'circle_turtle = py_basic_examples.circle_turtle:main',
        ],
    },
)
