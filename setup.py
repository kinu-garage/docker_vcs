from setuptools import find_packages
from setuptools import setup

package_name = "docker_vcs_tools"

setup(
    version="0.1.0",    
    author="Isaac Saito",
    author_email="iisaito.saito@gmail.com",
    classifiers=[
        "Development Status :: 1 - Planning",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: POSIX :: Linux",        
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.4",
        "Programming Language :: Python :: 3.5",
    ],
    description="A Python package that provides convenience utility for Docker, vcstool (another tool that provides utility for VCS), as well as rosdep.",
    install_requires=["docker>=5.0.3",
                      "vcstool>=0.3.0",],
    name=package_name,
    maintainer_email="iisaito.saito@gmail.com",
    license="Apach2.0",
    packages=find_packages(),
    package_data={package_name: ["config/example/*"]},
    url="https://github.com/kinu-garage/docker_vcs",
)
