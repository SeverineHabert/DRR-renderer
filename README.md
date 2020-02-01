Simple code to generate Digitally Reconstructed Radiographs from MetaImage (.mhd) CT data

The code generates DRR images in a transparent and simple manner, the user can tune every parameter, such as view point angles, distance to CT, intrinsics of DRR view, step of raytracing, ...
The DRR generation takes 2 seconds on a Intel(R) Core(TM) i7-2600 CPU @ 3.40GHz 8 cores on Ubuntu 14.10.
The .mhd standard is not fixed, therefore it might happen that the CT header use other terms than the ones used in our code for header element such as Element Spacing,... This should be check before running the code.

[![DOI](https://zenodo.org/badge/88970535.svg)](https://zenodo.org/badge/latestdoi/88970535)

### Build instructions

* setup developer environment via docker


    cd Docker
    docker build -t drr .

* build DRRgenerator within docker


    docker run -it -v $MYCODE:/mycode drr /bin/bash
    cd /mycode
    cmake .;make install;

### Build instructions

* test out the executable


    DRRgenerator ../lola11-55.mhd ../lola11-55.raw  out.png 0 0 0 512 512 -1000

