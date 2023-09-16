from setuptools import find_packages
from setuptools import setup

name_pip_package = "docker_vcs"
name_package_lib = "docker_vcs_lib"

setup(
    version="0.1.3",    
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
    install_requires=["docker>=5.0.3",
                      "vcstool>=0.3.0",],
    name=name_pip_package,
    maintainer_email="iisaito.saito@gmail.com",
    license="Apach2.0",
    packages=find_packages(),
    package_data={name_package_lib: ["config/example/*"]},
    url="https://github.com/kinu-garage/docker_vcs",
)
