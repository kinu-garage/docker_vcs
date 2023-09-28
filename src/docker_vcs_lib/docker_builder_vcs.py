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

DESC_DBUILDER = """'docker build' command with 3 additional features.
                    1) clone arbitrary set of repos utilizing "vcstool",
                    2) automatically resolve dependency of the software (using "rosdep install", then
                    3) build and install (only catkin_tools is supported as of 2021/02).
                    The software to be cloned and built have to be Catkin/Colcon packages."""
MSG_ENTRYPT_NO_SUBST = "As of 2021/02 entrypoint is statically embedded in a Docker image as resolving entrypoint seems not easy."


class DockerBuilderVCS():
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

    @staticmethod
    def check_prerequisite(paths):
        """
        @brief: If an element of 'paths' is not null, then see if there is a file at the path. 
        @type: [str] 
        @raise FileNotFoundError: When any input files are not found at the given path.
        """
        not_found = []
        not_file = []
        logging.debug("Paths: {}".format(paths))
        for path in paths:
            if path:
                if not os.path.exists(path):
                    not_found.append(path)
                elif not os.path.isfile(path):
                    not_file.append(path)
        if (not_found or not_file):  # Either one is non-null
            raise FileNotFoundError("These file(s) user inputs are not found:\n\tNot found: {}\n\tNot a file: {}".format(not_found, not_file))

    @staticmethod
    def parse_build_result_dict(build_result):
        """
        @brief Decode the docker.APIClient.build() output that is decoded as
            a dict object.
        @param build_result: ["{str: str}"], which is a returned object of
            'docker.APIClient.build' with a 'decode=True'.
        @return [str] build_result decoded into a list of str.
        """
        TRIM_PREFIX = "{'stream': '"
        TRIM_SUFFIX = "'}"
        logging.debug("Received obj in parse_build_result_dict: {}".format(build_result))
        lines = []
        for line in build_result:
            logging.debug("Received line: {}".format(line))
            #if "{'stream': '\n'}" == line:
                # Skipping a line that only contains meaningless info.
            #    continue
            logging.debug("Type of obj: {}".format(type(line)))
            logging.debug("keys: {}, vals: {}".format(line.keys(), line.values()))
            for k, v in line.items():
                # Trim JSON pre/suffixes.
                # TODO This impl isn't clean nor versatile yet.
                #line_clean = v[v.index(TRIM_PREFIX)+len(TRIM_PREFIX):v.index(TRIM_SUFFIX)-1]

                # Skipping meaningless line.
                if '\n' == v:
                    continue
                lines.append(v)
        return lines
                
    @staticmethod
    def _consturct_buildargs(args):
        """
        @summary: Build a Python dict that can be passed to 'Dockerfile'.
        @type args: Return value of 'argparse.ArgumentParser.parse_args()'
        @rtype: dict
        """
        buildargs={"BASE_DIMG": args.docker_base_img,
                   "ENTRY_POINT": args.entrypoint_exec,
                   "WS_IN_CONTAINER": args.workspace_in_container}
        if args.path_repos_file:
            buildargs["PATH_REPOS_FILE"] = os.path.basename(args.path_repos_file)
        return buildargs

    def _docker_build_low_api(
        self,
        path_dockerfile,
        baseimg,
        path_repos_file,
        network_mode="bridge",
        outimg="",
        rm_intermediate=True,
        path_context=".",
        entrypt_bin="",
        tmpwork_dir="/tmp",
        debug=False):
        """
        @brief Execute docker Python API via its lower-level API.
        @param path_context: Path of the directory'path_dockerfile' is located in.
        @return 
        """
        result = False
        dockerfile = os.path.basename(path_dockerfile)
        logging.info("Current dir before docker build: {}, dockerfile: {}".format(os.path.abspath(os.path.curdir), dockerfile))
        buildargs = self._consturct_buildargs(self._runtime_args)

        docker_api = docker.APIClient()

        #fobj = BytesIO(dockerfile.encode('utf-8'))
        responses = [line for line in docker_api.build(
            buildargs=buildargs,
            dockerfile=dockerfile,
        #    fileobj=fobj,
            network_mode=network_mode,
            path=tmpwork_dir,
            quiet=debug,
            rm=rm_intermediate,
            tag=outimg,
            decode=True  # to return a dict obj.
        )]
        logging.debug("docker build responses: {}".format(responses))
        #str_responses = self.parse_build_result(responses)
        return responses

    def docker_build(
            self,
            path_dockerfile,
            baseimg,
            network_mode="bridge",
            path_repos_file="",
            outimg="",
            rm_intermediate=True,
            entrypt_bin="entry_point.bash",
            path_docker_context="",
            tag=DEfAULT_DOCKERTAG,
            debug=False):
        """
        @brief Run 'docker build' using 'lower API version' https://docker-py.readthedocs.io/en/stable/api.html#module-docker.api.build.
            See also  https://github.com/docker/docker-py/issues/1400#issuecomment-273682010 for why lower API.
        @param tag: Tag for the Docker image to be built.
        @param entrypt_bin: Not implemented yet. Needs implemented to inject entrypoint in the built Docker img.
        """
        # Some verification for args.
        if not path_dockerfile:
            #raise ValueError("Missing value for '{}'".format(path_dockerfile))
            path_dockerfile = "./{}".format(DockerBuilderVCS.DEfAULT_DOCKERFILE)  # TODO if docker py sets default val then this won't be needed.
        if not outimg:
            outimg = baseimg + "_out"  # TODO make this customizable

        if entrypt_bin:
            logging.warning(MSG_ENTRYPT_NO_SUBST)

        _log = None
        try:
            #dimg, _log = self._docker_build_oo(
            _log = self._docker_build_low_api(
                path_dockerfile, baseimg,
                network_mode=network_mode,
                path_repos_file=path_repos_file,
                outimg=outimg, rm_intermediate=False, tmpwork_dir=path_docker_context, debug=debug)
            logging.debug("'docker build' is complete. Log: {}".format(_log))
        except docker.errors.BuildError as e:
            logging.error("'docker build' failed: {}".format(str(e)))
            #_log = e.build_log
            raise e
        except Exception as e:
            logging.error("'docker build' failed: {}".format(str(e)))
            raise e
        finally:
            res_lines = DockerBuilderVCS.parse_build_result_dict(_log)
            line_counter = 0
            for line in res_lines:
                logging.info("Line#{}: {}".format(line_counter, line))
                line_counter += 1

    def docker_readlog(self, logobj):
        """
        @summary Tentative method to read logs returned from some Docker Py API methods that return JSON w/o line break.
        """
        for line in logobj:
            if 'stream' in line:
                # TODO Not necessarilly logging.error
                logging.error(line['stream'].strip())

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
        entrypt_bin_abs = ""
        if entrypt_bin:
            entrypt_bin_abs = shutil.which(os.path.abspath(entrypt_bin))
        else:
            logging.warn("""
                    Entrypoint executable not passed, nor any files with 
                    commonly used names are not found. This doesn't mean the tool should fail,
                    e.g. if entrypoint is not used in Dockerfile, this won't
                    cause any error. Limitation tracked: https://github.com/130s/docker_vcs/issues/5
                    """)
        tobe_copied_into_container = [path_dockerfile, entrypt_bin_abs]
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

    def main(self):
        _MSG_LIMITATION_VOLUME = ("For now this cannot be defined multiple times, as opposed to 'docker run' where multiple '-v's can be passed to."
                                  "Therefore the value passed to this needs to be the ***top directory*** of all source folders to be built, e.g. 'src'.")

        parser = argparse.ArgumentParser(description=DESC_DBUILDER)
        ## For now assume dockerfile
        # Optional args
        parser.add_argument("--debug", help="Disabled by default.", action="store_true")
        parser.add_argument("--docker_base_img", help="Image Dockerfile begins with.", default=DockerBuilderVCS.DEfAULT_DOCKER_IMG)
        parser.add_argument("--dockerfile", help="Dockerfile path to be used to build the Docker image with. This can be remote. Default is './{}'.".format(DockerBuilderVCS.DEfAULT_DOCKERFILE))
        parser.add_argument("--entrypoint_exec", help="Docker's entrypoint that will be passed to Dockerfile.", default="")
        parser.add_argument("--log_file", help="If defined, std{out, err} will be saved in a file. If not passed output will be streamed.", action="store_true")
        parser.add_argument("--network_mode", help="Same options are available for networking mode for the 'docker run' command", default="bridge")
        parser.add_argument("--docker_out_img", help="Entire path of the Docker image ot be built.", default=".")
        parser.add_argument("--push_cloud", help="If defined, not pushing the resulted Docker image to the cloud.", action="store_false")
        parser.add_argument("--rm_intermediate", help="If False, the intermediate Docker images are not removed.", action="store_true")
        parser.add_argument("--tmp_context_path", help="Absolute path for the temporary context path docker_vcs creates (TBD we need a specific name for that). This option can save exec time, and is primarily helpful in docker_vcs' subsequent runs after the 1st run where you don't want to keep generating the temp folder. Double-quote the value from bash console.", default="")
        parser.add_argument("--workspace_in_container", help="Workspace where the software obtained from vcs will be copied into. Also by default install space will be under this dir.", default="/cws")
        parser.add_argument("--workspace_on_host", help="Current dir, as Docker's context shouldn't change in order for Dockerfile to be accessible.", default=".")
        gr_src_to_build = parser.add_mutually_exclusive_group()
        gr_src_to_build.add_argument("--path_repos_file", help="Path to .repos file to clone and build in side the container/image.")
        gr_src_to_build.add_argument("--volume", help="""
                               Not implemented yet. Bind volume mount. Unlike '--volume_build' option, this doesn't do anything but mounting.
                               Anything you want to happen can be defined in your 'Dockerfile'.
                               {}""".format(_MSG_LIMITATION_VOLUME))
        gr_src_to_build.add_argument("--volume_build", help="""
                               The path to be bound as volume mount. Sources in this path will be targeted to build into Docker container.
                               {} {}""".format(_MSG_LIMITATION_VOLUME, self._MSG_LIMITATION_SRC_LOCATION))

        args = parser.parse_args()
        self.init(args)
    #    dockerbuilder.init(
    #        debug=args.debug,
    #        log_file=args.log_file,
    #        push_cloud=args.push_cloud,
    #    )
        return self.generate_dockerimg(
            base_docker_img=args.docker_base_img,
            network_mode=args.network_mode,
            outimg=args.docker_out_img,
            path_dockerfile=args.dockerfile,
            debug=args.debug,
            entrypt_bin=args.entrypoint_exec,
            tmp_context_path=args.tmp_context_path)
