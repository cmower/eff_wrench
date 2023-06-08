from setuptools import setup

package_name = "eff_wrench"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Christopher E. Mower",
    maintainer_email="christopher.mower@kcl.ac.uk",
    description="ROS 2 node for estimating end-effector wrench.",
    license="GNU General Public License v3.0",
    install_requires = ["pyoptas", "numpy"],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["estimate_wrench = eff_wrench.estimate_wrench:main"],
    },
)
