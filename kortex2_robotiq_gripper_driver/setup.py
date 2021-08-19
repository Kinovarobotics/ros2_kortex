from setuptools import setup

package_name = 'kortex2_robotiq_gripper_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bren',
    maintainer_email='bren@brenpierce.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotiq_gripper_driver_85_action_server = kortex2_robotiq_gripper_driver.robotiq_gripper_driver_85_action_server:main',
        ],
    },
)
