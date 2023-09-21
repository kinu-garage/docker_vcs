# Copyright (C) 2023 Kinu Garage
# Licensed under Apache 2

from pathlib import Path
from setuptools import find_packages
from setuptools import setup

# For pkg discription https://packaging.python.org/en/latest/guides/making-a-pypi-friendly-readme/
this_directory = Path(__file__).parent
long_description_file = (this_directory / "README.md").read_text()

name_pip_package = "docker_vcs"
name_package_lib = "docker_vcs_lib"

setup(
    version="0.1.14",    
    author="Isaac Saito",
    author_email="iisaito.saito@gmail.com",
    classifiers=[
        "Development Status :: 1 - Planning",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: POSIX :: Linux",        
        "Programming Language :: Python :: 3",
    ],
    entry_points={
        'console_scripts': [
            'docker_vcs = docker_vcs_lib.docker_vcs:main'
        ],
    },    
    description="A Python package that provides convenience utility for Docker, vcstool (another tool that provides utility for VCS), as well as rosdep.",
    install_requires=["docker",
                      "sh",
                      "vcstool",],
    name=name_pip_package,
    maintainer_email="iisaito.saito@gmail.com",
    license="Apach2.0",
    long_description=long_description_file,
    long_description_content_type="text/markdown",
    #packages=find_packages(),
    package_data={name_package_lib: ["config/example/*"]},
    package_dir={'': 'src'},
    url="https://github.com/kinu-garage/docker_vcs",
)
