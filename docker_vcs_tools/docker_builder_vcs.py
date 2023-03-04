#! /usr/bin/env python3

# Copyright (C) 2021 Kinu Garage
# Licensed under Apache 2

import argparse
import datetime
import distutils.errors
from distutils.dir_util import copy_tree
import docker
from io import BytesIO
import json
import logging
import os
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


class OsUtil():
    """
    @summary: Utility for Operating System handling.
    """
    SUFFIX_BACKUP = ".bk"

    @staticmethod
    def copy_prop_file(path_src, path_dest):
        """
        @brief: Python's file copy methods are known to be missing an option
            to copy meta data. This method copies them from a file to another.
        @return: True if copying metadata was successful.
        @raise AssertionError: When either UID / GID / file size / st_dev
            differs b/w src and dest files.
        """
        meta_src = os.stat(path_src)
        os.chown(path_dest, meta_src.st_uid, meta_src.st_gid)

        meta_dest = os.stat(path_dest)
        if (meta_src.st_uid != meta_dest.st_uid) or \
           (meta_src.st_gid != meta_dest.st_gid) or \
           (meta_src.st_dev != meta_dest.st_dev) or \
           (meta_src.st_size != meta_dest.st_size):
            raise AssertionError("Copying meta data failed. Metadata per file:\n\tSrc: {}\n\tDst: {}".format(meta_src, meta_dest))
        else:
            return True

    @staticmethod
    def copy(src_path,
             path_root_backup="",
             dest_file_extension=SUFFIX_BACKUP,
             errmsg="No right to write in the destination",
             copy_metadata=True):
        """
        @brief: Make a copy of the given file in the given folder.
            Meta data of the file can be copied as well.
        @param src_path: Path of the source file. If this points to a directory,
            then the entire directory will be copied.
        @param path_root_backup: (Option) Root path of the backup file to be saved in.
        @param dest_file_extension: (Option) file extension of the backup file.
            This is only used when 'path_root_backup' is not zero value.
        @param copy_metadata: If True, metadata will be copied
        @return Absolute path of the destination.
        @raise EnvironmentError When the path 'src_path' cannot be found.
        @raise PermissionError when saving a file failed due to permission.
        """
        # Some screening in the beginning
        if not os.path.exists(src_path):
            raise EnvironmentError("Path defined in src_path '{}' cannot be found.".format(src_path))

        src_rootdir = os.path.dirname(src_path)
        # When relative path to a file in the working directory is passed,
        # abs path needs to be figured out.
        if not src_rootdir:
            src_rootdir = os.path.abspath(os.getcwd())
        # "leaf" here is the last segment of the path. 
        # It's the name of either directory or file.
        src_leaf_name = pathlib.Path(src_path).parts[-1]
        dest_abs_path = ""
        if path_root_backup:
            if not os.path.exists(path_root_backup):
                os.makedirs(path_root_backup)
            dest_abs_path = os.path.join(path_root_backup, src_leaf_name)
        else:
            dest_abs_path = os.path.join(src_rootdir, src_leaf_name + dest_file_extension)
        logging.debug("Paths\n\tpath_root_backup: {}\n\tsrc_rootdir = {}\n\tsrc_leaf_name = {}\n\tdest_abs_path = {}".format(
            path_root_backup, src_rootdir, src_leaf_name, dest_abs_path))

        try:
            logging.info("Saving from '{}', to '{}'".format(src_path, dest_abs_path))
            if os.path.isfile(src_path):
                shutil.copyfile(src_path, dest_abs_path)
            else:
                logging.info("'{}' is not a file. Copying it to '{}'".format(src_path, dest_abs_path))
                shutil.copytree(src_path, dest_abs_path)
        except PermissionError as e:
            raise type(e)(e.message + " Copy failed due to the permission error at '{}'. ".format(src_path))

        if copy_metadata:
            try:
                OsUtil.copy_prop_file(src_path, dest_abs_path)
            except AssertionError as e:
                logging.fatal("File was copied, but copying metadata failed. Detail: {}".format(str(e)))

        return dest_abs_path


class DockerBuilderVCS():
    """
    @summary See DESC_DBUILDER variable.
    """
    DEfAULT_DOCKER_IMG = "ubuntu:focal"
    DEfAULT_DOCKERFILE = "Dockerfile_ros1"
    DEfAULT_DOCKERTAG = "dockerimg_built"
    DEfAULT_ENTRYPOINT_PTN = "entry*point.*sh"
    TMPCONTEXT_PREFIX = "docker_vcs"
    TOPDIR_SRC = "src"  # This folder name seems requirement for Catkin
    _MSG_LIMITATION_SRC_LOCATION = "There may be usecases to use both 'volume_build' and 'path_repos', which is not yet covered as the need is unclear."

    def __init__(self):
        try:
            self._docker_client = docker.from_env()
        except (docker.errors.APIError, docker.errors.TLSParameterError) as e:
            logging.error(str(e))
            exit(1)
        pass

    def docker_login():
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

    @staticmethod
    def check_prerequisite(paths):
        """
        @type: [str] 
        @raise FileNotFoundError: When any input files are not found at the given path.
        """
        not_found = []
        for path in paths:
            if path and (not os.path.isfile(path)):
                not_found.append(path)
        if not_found:
            raise FileNotFoundError("These file(s) user inputs are not found: {}".format(not_found))

    @staticmethod
    def parse_build_result(build_result):
        """
        @brief Decode the docker.APIClient.build() output, which has a tricky
            data structure, to a list of str.
         
        @param build_result: [b"{str: str}]
            A raw output from 'docker.APIClient.build' ('low-level API") may
            look something like:
                [b'{"stream":"Step 1/33 : ARG BASE_DIMG"}\r\n{"stream":"\\n"}\r\n
                :
                /etc/apt/sources.list.d/gazebo-latest.list  \\u0026\\u0026 apt-get clean \\u0026\\u0026 rm -rf /var/lib/apt/lists/*\' returned a non-zero code: 100"},"error":"The command \'/bin/bash -c apt-get update  \\u002 \\"deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $UBUNTU_DISTRO main\\" \\u003e /etc/apt/sources.list.d/gazebo-latest.list  \\u0026\\u0026 apt-get clean \\u0026\\u0026 rm -rf /var/lib/apt/lists/*\' returned a non-zero code: 100"}\r\n']
        @return [str] build_result decoded into a list of str.
        """
        list_str = []
        for dict_byte in build_result:
            ed = dict_byte.split(b'\r\n')
            for raw_decoded_str in ed:
                if raw_decoded_str == '{"stream":"\n"}':
                    continue
                list_str.append(raw_decoded_str.decode()) 
        return list_str

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
            path=tmpwork_dir,
            quiet=debug,
            rm=rm_intermediate,
            tag=outimg,
            decode=True  # to return a dict obj.
        )]
        logging.debug("docker build responses: {}".format(responses))
        #str_responses = self.parse_build_result(responses)
        str_responses = responses
        if str_responses:
            result = True
        res_lines = DockerBuilderVCS.parse_build_result_dict(str_responses)
        line_counter = 0
        for line in res_lines:
            logging.info("Line#{}: {}".format(line_counter, line))
            line_counter += 1
        return result
        
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

        img, _log = self._docker_client.images.build(
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
        return img, _log
    
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
            dimg, _log = self._docker_build_low_api(
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
            if _log:
                for line in _log:
                    if 'stream' in line:
                        logging.error(line['stream'].strip())

    def docker_readlog(self, logobj):
        """
        @summary Tentative method to read logs returned from some Docker Py API methods that return JSON w/o line break.
        """
        for line in logobj:
            if 'stream' in line:
                # TODO Not necessarilly logging.error
                logging.error(line['stream'].strip())

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
                loggin.warn("""
                    Entrypoint executable not passed, nor any files with 
                    commonly used names are not found. This doesn't mean error,
                    e.g. if entrypoint is not used in Dockerfile, this won't
                    cause any error. Limitation tracked: https://github.com/130s/docker_vcstool/issues/5
                    \n{}""".format(str(e)))
        # If prerequisite not met, exit the entire process.
        try:
            self.check_prerequisite([path_dockerfile, entrypt_bin])
        except FileNotFoundError as e:
            logging.error(str(e))
            exit(1)

        #self.docker_login()

        # Copy the resources that are meant to be copied into
        # the to-be-generated Docker image into the temp location.
        tobe_copied = [path_dockerfile, entrypt_bin]
        tmp_context_path = "/tmp/docker_vcs_{}".format(
            datetime.datetime.fromtimestamp(
                time.time()).strftime('%Y%m%d%H%M%S'))
        if self._runtime_args.path_repos_file:
            tobe_copied.append(self._runtime_args.path_repos_file)
        elif self._runtime_args.volume:
            raise NotImplementedError("'--volume' option is not yet implemented.")
        elif self._runtime_args.volume_build:
            # Despite the name 'volume', copy operation is done in this block.
            # That is because, 'docker build' along with this program does NOT
            # mount the volume specified unlike 'docker run'.
            logging.info(
                """Arg '--volume_build' is set. All files and folders under the
                volume dir '{}' are to be copied into the workspace in the Docker image.""".format(self._runtime_args.volume_build))
            if not self._runtime_args.volume_build.endswith(self.TOPDIR_SRC):
                logging.info(
                    """The name of the top directory at the mounted path '{}'
                    is not 'src', then not adding the top dir to tobe_copied.""".format(self._runtime_args.volume_build))
                for f in os.listdir(self._runtime_args.volume_build):
                    self.copy(f, tmp_docker_context_dir)
            tobe_copied.append(self._runtime_args.volume_build)
        # Copying files
        for f in tobe_copied:
            OsUtil.copy(f, tmp_context_path)
        #self.copy(tmp_context_path, tobe_copied)

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
        _MSG_LIMITATION_VOLUME = (
            """
            For now this cannot be defined multiple times, as opposed to
            'docker run' where multiple '-v's can be passed to. Therefore the
            value passed to this needs to be the ***top directory*** of all
            source folders to be built, e.g. 'src'.
            """)

        parser = argparse.ArgumentParser(description=DESC_DBUILDER)
        # Optional but close to required args
        parser.add_argument("--docker_base_img", help="Image Dockerfile begins with.", default=DockerBuilderVCS.DEfAULT_DOCKER_IMG)
        parser.add_argument("--docker_out_img", help="Entire path of the Docker image ot be built.", default=".")
        parser.add_argument("--dockerfile", default=DockerBuilderVCS.DEfAULT_DOCKERFILE,
                            help="Dockerfile path to be used to build the Docker image with. This can be remote. Default is './{}'.".format(DockerBuilderVCS.DEfAULT_DOCKERFILE))
        parser.add_argument("--workspace_in_container", help="Workspace where the software obtained from vcs will be copied into. Also by default install space will be under this dir.", default="/cws")
        parser.add_argument("--workspace_on_host", help="Current dir, as Docker's context shouldn't change in order for Dockerfile to be accessible.", default=".")
        gr_src_to_build = parser.add_mutually_exclusive_group()
        gr_src_to_build.add_argument("--path_repos_file",
                                     help="""
                                     Path to .repos file to clone and build in side the container.
                                     Mutually exclusive with '--volume*'.""")
        gr_src_to_build.add_argument("--volume", help="""
                               Not implemented yet. Bind volume mount.
                               Unlike '--volume_build' option, this doesn't do
                               anything but mounting. Useful for passing resource
                               that is needed during the development onto a
                               container (Anything you want to happen inside a
                               container during build time can be defined in
                               your 'Dockerfile'. Mutually exclusive with
                               '--path_repos_file' and '--volume_build'.{}""".format(_MSG_LIMITATION_VOLUME))
        gr_src_to_build.add_argument("--volume_build", help="""
                               The files/directories in the given path will be
                               copied into the resulted Docker image. 'volume'
                               may be a misleading name. {} {}""".format(_MSG_LIMITATION_VOLUME, self._MSG_LIMITATION_SRC_LOCATION))
        # Purely optional args
        parser.add_argument("--debug", help="Disabled by default.", action="store_true")
        parser.add_argument("--entrypoint_exec", help="Docker's entrypoint that will be passed to Dockerfile.", default=".")
        parser.add_argument("--log_file", help="If defined, std{out, err} will be saved in a file. If not passed output will be streamed.", action="store_true")
        parser.add_argument("--network_mode", help="Same options are available for networking mode for the 'docker run' command", default="bridge")
        parser.add_argument("--push_cloud", help="If defined, not pushing the resulted Docker image to the cloud.", action="store_false")
        parser.add_argument("--rm_intermediate", help="If False, the intermediate Docker images are not removed.", action="store_true")
        parser.add_argument("--tmp_context_path",
                            help="""
                            Absolute path for the temporary context path
                            docker_vcs creates (TBD we need a specific name for
                            that). This option can save exec time, and is
                            primarily helpful in docker_vcs' subsequent runs
                            after the 1st run where you don't want to keep
                            generating the temp folder. Double-quote the value
                            from bash console.""", default="")

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


if __name__ == '__main__':
    dockerbuilder = DockerBuilderVCS()
    dockerbuilder.main()
