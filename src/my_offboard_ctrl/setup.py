from setuptools import find_packages, setup

package_name = 'my_offboard_ctrl'

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
    maintainer='cre',
    maintainer_email='cre@todo.todo',
    description='TODO: Package description',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_ctrl_example = my_offboard_ctrl.offboard_ctrl_example:main',
            'flight=my_offboard_ctrl.flight:main',
            'langcontrol=my_offboard_ctrl.langcontrol:main',
            'realcontrol=my_offboard_ctrl.realcontrol:main',
            'testaction=my_offboard_ctrl.testaction:main',
            'testaction1=my_offboard_ctrl.testaction1:main',
            'testaction2=my_offboard_ctrl.testaction2:main',
            'testhead=my_offboard_ctrl.testhead:main',
            'testhead1=my_offboard_ctrl.testhead1:main',
            'testhead2=my_offboard_ctrl.testhead2:main',
            'test=my_offboard_ctrl.test:main',
            'drone_unified = my_offboard_ctrl.drone_unified:main',
        ],
    },
)
