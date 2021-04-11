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
from docker_builder_vcs import DockerBuilderVCS


class TestDockerBuilderVCS():

    def test_copy(self):
        """
        @summary: TBD
        """
        tmp_dir = "/tmp/{}".format(datetime.datetime.fromtimestamp(time.time()).strftime('%Y%m%d%H%M%S'))
        list_files_src = []
        path_file_aaa = "/tmp/file.aaa" 
        Path(path_file_aaa).touch()
        path_file_bbb = "/tmp/file.bbb"
        Path(path_file_bbb).touch()
        path_folder_aaa = "/tmp/folder_aaa"
        os.makedirs(path_folder_aaa, exist_ok=True)
        list_files_src.append(path_file_aaa)
        list_files_src.append(path_file_bbb)
        list_files_src.append(path_folder_aaa)

        dbv = DockerBuilderVCS()
        ret = dbv.copy(tmp_dir, list_files_src)
        assert isinstance(ret, str)
