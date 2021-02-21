#! /usr/bin/env python3

# Copyright (C) 2021 Kinu Garage
# Licensed under TBD

import docker
from docker.errors import APIError, TLSParameterError
import logging
import os
import sh
import shutil
import subprocess

_DEfAULT_DOCKERFILE = "Dockerfile"


class DockerCatkinPkgBuilder():
    """
    @summary 'docker build' command with 3 additional features.
        1) clone arbitrary set of repos, 
        2) automatically resolve dependency of the software, then
        3) build and install.
        The software to be cloned and built have to be Catkin/Colcon packages.
    """
    DEfAULT_DOCKERFILE = "Dockerfile"

    def __init__(self):
        pass

    def init(self,
        debug=False,
        dockerimg_base="ros:noetic-ros-base",
        dockerfile_name=_DEfAULT_DOCKERFILE,
        log_file=False,
        no_push_cloud=False,
        temp_workdir_container="/root/tobedeleted/",
        temp_workdir_host=".",  # Current dir, as Docker's context shouldn't change in order for Dockerfile to be accessible.
        ):

        loglevel = logging.INFO
        log_path_file = ''  # TODO Make this passable
        if debug:
            loglevel = logging.DEBUG
        if log_file:
            log_path_file = '/tmp/dockerbuilder.log'
        logging.basicConfig(filename=log_path_file, level=loglevel)

        # Check required args
        logging.debug("These are what's passed: docker_registry: {}".format(docker_registry))
        if not (docker_registry):
            logging.error("Missing Docker auth info. Exiting.\n These are what's passed: docker_registry: {}".format(docker_registry))
            exit(1)

        #self.docker_login()

        # TODO Should be done in less dirtier way e.g. **kwargs.
        # TODO Also move this portion to main to make API backward comp. 
        self._dockerimg_base = dockerimg_base
        self._dockerfile_name = dockerfile_name
        self._temp_workdir_container = temp_workdir_container
        self._temp_workdir_host = temp_workdir_host
        self._docker_registry = docker_registry
        self._no_push_cloud = no_push_cloud
        self._userid_localrunner = userid_localrunner

    def docker_build(self, baseimg, path_dockerfile):
        # Gave up passing /dev during build https://stackoverflow.com/c/plusonerobotics/questions/172, ditching docker-compose either. Came back to using 'docker build'
        try:
            img, _log = self._docker_client.images.build(
                tag=baseimg,
                path=".",  # TODO Can we really assume current dir is always the dir this pg runs on and where Dockerfile is present?
                dockerfile=path_dockerfile,
                rm=True,
                buildargs={"temp_workdir_container": self._temp_workdir_container,
                           "dockerimg_noenc": self._dockerimg_noenc},
                quiet=False)
        except docker.errors.BuildError as e:
            logging.error("'docker build' failed: {}".format(str(e)))
            _log = e.build_log
            raise
        except Exception as e:
            logging.error("'docker build' failed: {}".format(str(e)))
            raise e
        finally:
            for line in _log:
                if 'stream' in line:
                    logging.error(line['stream'].strip())
        
    def check_prerequisite(self):
#        if not shutil.which("aws"):
#            raise RuntimeError("awscli not found to be installed. Exiting.")
        if not os.path.exists(self._temp_workdir_host): os.mkdir(self._temp_workdir_host)  # Temp workdir (on host)

    def docker_readlog(self, logobj):
        """
        @summary Tentative method to read logs returned from some Docker Py API methods that return JSON w/o line break.
        """
        for line in logobj:
            if 'stream' in line:
                # TODO Not necessarilly logging.error
                logging.error(line['stream'].strip())

    def main(self, ):
        logging.info("[docker] Pulling an image: {}".format(self._dockerimg_base))
        self._docker_client.images.pull(self._dockerimg_base)

        # If prerequisite not met, exit the entire process.
        try:
            self.check_prerequisite()
        except RuntimeError as e:
            logging.error(str(e))
            exit(1)

        self.docker_build(self._dockerimg_base)

        return True


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("dockerfile", help="Dockerfile to build the Docker image with. Default is '{}'.".format(DockerCatkinPkgBuilder.DEfAULT_DOCKERFILE))
    parser.add_argument("path_repos", help="Path to .repos file to clone and build in side the container/image.")
    # Optional args
    parser.add_argument("--debug", help="Disabled by default.", action="store_true")
    parser.add_argument("--dockerimg_base", default="ros:noetic-ros-base",
                        help="Full path Docker image tag to get un-encrypted files from. It can be the Docker image that was built in an automated pipeline.")
    parser.add_argument("--log_file", help="If defined, std{out, err} will be saved in a file. If not passed output will be streamed.", action="store_true")
    parser.add_argument("--no_push_cloud", help="If defined, not pushing the resulted Docker image to the cloud.", action="store_true")

    args = parser.parse_args()
    dockerbuilder = DockerCatkinPkgBuilder()
    dockerbuilder.init(
        debug=args.debug,
        log_file=args.log_file,
        no_push_cloud=args.no_push_cloud,
    )
    if not dockerbuilder.main(args.dockerimg_base, args.path_repos):
        exit(1)
