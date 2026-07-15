from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "hairoom_control"
package_dir = Path(__file__).resolve().parent
repo_root = Path(__file__).resolve().parents[2]


def existing(pattern):
    return [str(path) for path in glob(str(pattern)) if Path(path).exists()]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", existing(package_dir / "config" / "*.yaml")),
        (f"share/{package_name}/launch", existing(package_dir / "launch" / "*.launch.py")),
        (f"share/{package_name}/data", existing(repo_root / "data" / "points.csv")),
        (f"share/{package_name}/meshes", existing(repo_root / "src" / "meshes" / "*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hairoom",
    maintainer_email="dev@hairoom.local",
    description="HTTP control surface for Hairoom robot procedures backed by MoveIt2.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "wash_server = hairoom_control.server:main",
        ],
    },
)
