from setuptools import setup

package_name = "kortex2_robotiq_gripper_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="brennand pierce",
    maintainer_email="bren.pierce@picknik.ai",
    entry_points={
        "console_scripts": [
            "robotiq_gripper_driver_85_action_server = kortex2_robotiq_gripper_driver.robotiq_gripper_driver_85_action_server:main",
        ],
    },
)
