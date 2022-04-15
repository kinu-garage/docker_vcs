#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2021 Kinu Garage

import datetime
import logging
import os
from pathlib import Path
import pytest
import time

# Hacky/inappropriate way to import.
from docker_builder_vcs import OsUtil


class TestOsUtil():
    def test_copy(self):
        """
        @summary: TBD
        """
        tmp_dir = "/tmp/{}_{}".format(
            type(self).__name__,
            datetime.datetime.fromtimestamp(time.time()).strftime('%Y%m%d%H%M%S'))
        logging.info("tmp_dir = {}".format(tmp_dir))
        list_files_src = []
        path_folder_src = os.path.join(tmp_dir, "folder_src")
        path_folder_dest = os.path.join(tmp_dir, "folder_dest")
        path_file_aaa = os.path.join(path_folder_src, "file.aaa") 
        path_file_bbb = os.path.join(path_folder_src, "file.bbb")
        path_folder_ccc = os.path.join(path_folder_src, "folder.ccc")

        # Making dirs and files
        os.makedirs(tmp_dir, exist_ok=True)
        os.makedirs(path_folder_src, exist_ok=True)
        os.makedirs(path_folder_dest, exist_ok=True)
        Path(path_file_aaa).touch()
        Path(path_file_bbb).touch()
        os.makedirs(path_folder_ccc, exist_ok=True)
        list_files_src.append(path_file_aaa)
        list_files_src.append(path_file_bbb)
        list_files_src.append(path_folder_ccc)

        logging.warning("Src dir: {}\nDest dir: {}".format(
            os.listdir(path_folder_src), os.listdir(path_folder_dest)))

        for f in list_files_src:
            dest_abs_path = OsUtil.copy(f, path_folder_dest)
            # Verify if each dest file exists.
            assert os.path.exists(dest_abs_path)

        # Verify the num of files are the same b/w src and dest dirs.
        assert len(os.listdir(path_folder_src)) == len(os.listdir(path_folder_dest))
