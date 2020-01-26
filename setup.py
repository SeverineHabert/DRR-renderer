import sys

from skbuild import setup

setup(
    name="DRRgenerator",
    version="0.1",
    description="DRR-renderer",
    author='Severine Habert',
    license="MIT",
    packages=[],
    install_requires=[],
)

# copy built executable to bin folder.

import shutil,os
src_file = '_skbuild/linux-x86_64-3.7/cmake-install/bin/DRRgenerator'
tgt_file = 'bin/DRRgenerator'
if os.path.exists(src_file):
    shutil.copy(src_file,tgt_file)