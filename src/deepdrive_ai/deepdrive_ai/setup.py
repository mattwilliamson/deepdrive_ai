from setuptools import find_packages, setup
import glob

package_name = "deepdrive_ai"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/audio", glob.glob("audio/*")),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Matt Williamson",
    maintainer_email="matt@aimatt.com",
    description="TODO: Package description",
    license="GPL-3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chat_bot_node = deepdrive_ai.chat_bot_node:main",
            "langchain_test = deepdrive_ai.langchain_test:main",
        ],
    },
)
