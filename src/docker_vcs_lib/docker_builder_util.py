#! /usr/bin/env python3

# Copyright (C) 2021 Kinu Garage
# Licensed under Apache 2

import argparse
import datetime
import distutils.errors
from distutils.dir_util import copy_tree
import docker
from docker_vcs_lib.os_util import OsUtil
from io import BytesIO
import json
import logging
import os
from requests.exceptions import ConnectionError
import pathlib
import sh
import shlex
import shutil
import time

class DockerBuilderUtil():
    """
    @summary See DESC_DBUILDER variable.
    """
    DEfAULT_DOCKER_IMG = "ubuntu:focal"
    DEfAULT_DOCKERFILE = "Dockerfile"
    DEfAULT_DOCKERTAG = "dockerimg_built"
    DEfAULT_ENTRYPOINT_PTN = "entry*point.*sh"
    TMPCONTEXT_PREFIX = "docker_vcs"
    TOPDIR_SRC = "src"  # This folder name seems requirement for Catkin
    _MSG_LIMITATION_SRC_LOCATION = "There may be usecases to use both 'volume_build' and 'path_repos', which is not yet covered as the need is unclear."

    def __init__(self):
        try:
            self._docker_client = docker.from_env()
        except (docker.errors.DockerException, docker.errors.APIError, docker.errors.TLSParameterError, ConnectionError) as e:
            # Message hinted from https://github.com/osrf/rocker/pull/215
            msg = ("Docker Client failed to connect to docker daemon."
                   " Please verify that docker is installed and running,"
                   " as well as that you have permission to access the docker daemon."
                   " This happens usually by being a member of the docker group,"
                   " also if this is run inside a Docker container (this module is intended to run outside of a container)."
                   " Underlying error was:\n{}".format(str(e)))
            logging.error(msg)
            exit(1)
        pass

    def docker_login():
        """
        @deprecated Context for this method is forgotten.
        """
        try:
            # When pushing to cloud, log in to docker registry.
            if self._push_cloud:
               self._docker_client.login(
                   username=docker_account, password=docker_pw, registry=docker_registry)
        except (APIError, TLSParameterError) as e:
            logging.error(str(e))
            exit(1)

    def init(self, args):
        """
        @summary: Convenient funtion to read the 'args' to set the values to
            the member variables, so that later other methods can be called
            by passing the from member variables.
        """
        # Set all other user inputs as a member variable.
        self._runtime_args = args
        loglevel = logging.INFO
        log_path_file = ''  # TODO Make this passable
        if self._runtime_args.debug:
            loglevel = logging.DEBUG
        if self._runtime_args.log_file:
            log_path_file = '/tmp/dockerbuilder.log'
        logging.basicConfig(filename=log_path_file, level=loglevel)

        # Required args
        self._dockerfile_name = self._runtime_args.dockerfile

        self._push_cloud = self._runtime_args.push_cloud
        self._temp_workdir_host = self._runtime_args.workspace_on_host
        self._workspace_in_container = self._runtime_args.workspace_in_container
#        self._docker_account = args.docker_account
#        self._docker_pw = args.docker_pw
#        self._docker_registry = args.docker_registry
#        logging.debug("These are what's passed: docker_registry: {}".format(docker_registry))
#        if not (docker_registry):
#            logging.error("Missing Docker auth info. Exiting.\n These are what's passed: docker_registry: {}".format(docker_registry))
#            exit(1)

        #self.docker_login()

        # TODO Also move this portion to main to make API backward comp.
        # TODO Whether to include Docker API here is debatable.
        
    def _docker_build_oo(
        self,
        path_dockerfile,
        baseimg,
        network_mode="bridge",
        path_repos_file="",
        outimg="",
        rm_intermediate=True,
        entrypt_bin="",
        tmpwork_dir="/tmp",
        debug=False):
        """
        @brief Execute docker Python API via its "Object-oriented API" i.e. non-low-level/RESTful API.
        @deprecated: Not planned to be maintained. Use docker_build instead. This uses docker-py's method that misses some capability. 
            - 20220324 What's wrong with the non-low-level API? All args passed to '_docker_build_low_api' can be taken by OO API AFAIK.
        @param network_mode: networking mode for the 'docker run' commands during build.
            Ref. https://docs.docker.com/engine/reference/run/#network-settings for available options.
        """
        buildargs = self._consturct_buildargs(self._runtime_args)

        _log = self._docker_client.images.build(
            dockerfile=os.path.basename(path_dockerfile),
            buildargs=buildargs,
            #path=tmpwork_dir,
            network_mode=network_mode,
            path=".",
            quiet=debug,
            rm=rm_intermediate,
            tag=outimg
        )
        # TODO Parse 'responses' object:  <itertools._tee object at 0x7fa75c026bc0>
        logging.info("docker build responses: {}".format(responses))        
        return _log
    
    def _docker_run(self, dimage, cmd, envvars=None):
        """
        @param cmd: A string of command that is passed to `bash -c`.
        @type envvars: Dict
        @return A container object (https://docker-py.readthedocs.io/en/stable/containers.html#docker.models.containers.Container).
        """
        try:
            container = self._docker_client.containers.run(
                image=dimage,
                command=["bash", "-c", '{}'.format(cmd)],
                stream=True,
                environment=envvars,
                privileged=True
            )
        except docker.errors.ContainerError as e:
            logging.error("'docker run' failed: {}".format(str(e)))
            self.docker_readlog(e.stderr)
            raise e
        return container

    def copy(self, src_list_files, dest_dir):
        """
        @deprecated: 2022/04/08 'OsUtil.copy' is prioritized for maintenance.
        @summary: If the some files (dockerfile, path_repos_file) that 'docker build' uses are not under current dir,
            1. copy them in the temp folder on the host.
            2. CD into the temp folder so that 'docker build' run in that context.
        @param src_list_files: Absolute or relative path to a folder or file to be copied. 
        """
        if not os.path.isdir(path_docker_context):
            os.makedirs(path_docker_context)
        for path in list_files_src:
            logging.debug("File to be copied: '{}'".format(path))
            if os.path.isdir(path):
                try:
                    distutils.dir_util.copy_tree(os.path.abspath(path), os.path.join(path_docker_context, path))
                except errors.DistutilsFileError as e:
                    logging.warn("Failed to copy an object. Moving on to continue copying the rest.:\n\t{}".format(str(e)))                    
            else:
                shutil.copy2(src_file_path, dest_dir)
        logging.info("Files are copied into a temp dir: '{}'".format(os.listdir(dest_dir)))
        return dest_dir

    def docker_build_from_mount(self, docker_img, path_volume, debug=False):
        """
        @summary Build source that is in mounted volume if anything.
           Docker container will be started with the volume(s) to be mounted. Then build.
        @type paths_volume: [str]
        """
        _TMP_WS_DIR = "/workspace"
        _TMP_WS_MOUNTED_DIR = os.path.join(_TMP_WS_DIR, "src/mounted") 
        envvars = "-v {}:{}".format(path_volume, _TMP_WS_SRC_DIR)
        cmd = "cd {} && colcon build".format(_TMP_WS_DIR)
        container = self._docker_run(docker_img, cmd, envvars)
        # commit the docker img with the same 'docker_img'
        container.commit(docker_img)
        # TODO Terminate the container?

    def generate_dockerimg(self,
              base_docker_img=DEfAULT_DOCKER_IMG,
              network_mode="bridge",
              outimg="",
              path_dockerfile="",
              debug=False,
              entrypt_bin="",
              tmp_context_path=""):
        """
        @param base_docker_img: Docker image that Dockerfile starts with. This must be supplied.
        @param entrypt_bin: Path to the executable used in a Dockerfile.
        """
        if not entrypt_bin:
            try:
                entrypt_bin = BuilderVCSUtil.get_path(entrypt_bin)
            except FileNotFoundError as e:
                logging.warn("""
                    Entrypoint executable not passed, nor any files with 
                    commonly used names are not found. This doesn't mean the tool should fail,
                    e.g. if entrypoint is not used in Dockerfile, this won't
                    cause any error. Limitation tracked: https://github.com/130s/docker_vcs/issues/5
                    \n{}""".format(str(e)))
        tobe_copied_into_container = [path_dockerfile, entrypt_bin]                
        # If prerequisite not met, exit the entire process.
        try:
            self.check_prerequisite(tobe_copied_into_container)
        except FileNotFoundError as e:
            logging.error(str(e))
            exit(1)

        #self.docker_login()

        # Copy the resources that are meant to be copied into
        # the to-be-generated Docker image into the temp location.

        # TODO Check if '--tmp_context_path' is a valid path.

        tmp_context_path = "/tmp/docker_vcs_{}".format(datetime.datetime.fromtimestamp(time.time()).strftime('%Y%m%d%H%M%S'))
        if self._runtime_args.path_repos_file:
            tobe_copied_into_container.append(self._runtime_args.path_repos_file)
        elif self._runtime_args.volume_build:
            logging.info("'volume_build' is set. All files and folders under the volume dir '{}' are to be copied into the workspace.".format(
                self._runtime_args.volume_build))
            if not self._runtime_args.volume_build.endswith(self.TOPDIR_SRC):
                logging.info("The mounted top dir name '{}' is not '{}', then not adding the top dir to tobe_copied_into_container.".format(
                    self._runtime_args.volume_build, self.TOPDIR_SRC))
                for f in os.listdir(self._runtime_args.volume_build):
                    OsUtil.copy(f, tmp_context_path)
            tobe_copied_into_container.append(self._runtime_args.volume_build)
        # Copying files
        for f in tobe_copied_into_container:
            OsUtil.copy(f, tmp_context_path)
        #self.copy(tmp_context_path, tobe_copied_into_container)

        # Someone said chdir-ing and running 'docker build' can be dangerous so
        # stop doing so. Instead, because 'docker build' can take the context
        # path, just utilize that.
        #os.chdir(tmp_context_path)

        # TODO if volume mount build, copy the files in the volume. 

        logging.debug("path_dockerfile: {}, base_docker_img: {}, path_repos_file: {}".format(
            path_dockerfile, base_docker_img, self._runtime_args.path_repos_file))
        self.docker_build(
            path_dockerfile,
            base_docker_img,
            path_repos_file=self._runtime_args.path_repos_file,
            network_mode=network_mode,
            outimg=outimg,
            rm_intermediate=False,
            entrypt_bin=entrypt_bin,
            path_docker_context=tmp_context_path,
            debug=debug)
        
        return True
