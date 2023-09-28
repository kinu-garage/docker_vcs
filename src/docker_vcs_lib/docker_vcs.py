#! /usr/bin/env python3

# Copyright (C) 2023 Kinu Garage LLC
# Licensed under Apache 2

from docker_vcs_lib.docker_builder_vcs import DockerBuilderVCS

def main():
    dockerbuilder = DockerBuilderVCS()
    dockerbuilder.main()

if __name__ == '__main__':
    main()
