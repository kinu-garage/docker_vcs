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
                # copyfile needs the destination path already exists, so making
                # one here if it doesn't exist. 
                if not os.path.exists(path_root_backup):
                    os.makedirs(path_root_backup)

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
