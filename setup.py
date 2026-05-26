from glob import glob

from setuptools import find_namespace_packages, setup


package_name = "visual_localization"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_namespace_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="TERBOUCHE Hacene",
    maintainer_email="hacene.terbouche@gmail.com",
    description="Visual localization for UAVs using satellite imagery.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "visual_localization_node = svl.ros2_node:main",
        ],
    },
)
