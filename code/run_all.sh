#!/usr/bin/env bash

# If project not ready, generate cmake file.
if [[ ! -d build ]]; then
    echo "good"
else
    rm -rf build
fi
mkdir -p build
cd build
cmake ..
make -j
cd ..

# Run all testcases.t
# You can comment some lines to disable the run of specific examples.
mkdir -p output
# bin/PA4 ptn testcases/smallpt.txt output/zg_qu.bmp nodebug
# bin/PA4 ppm testcases/water2.txt output/water____.bmp nodebug > h.txt
# bin/PA4 ppm testcases/zg_len.txt output/zg_len.bmp nodebug
# bin/PA4 ppm testcases/tuzi__.txt output/zg5.bmp nodebug > h.txt
bin/PA4 pt testcases/mov.txt output/mov_zg7.bmp nodebug > i.txt
# bin/PA4 ppm testcases/water2.txt output/water____.bmp nodebug > h.txt
# bin/PA4 ppm testcases/ppm.txt output/ppm_2.bmp nodebug
# bin/PA1 pt testcases/meili.txt output/meili_pt_2000.bmp nodebug
# bin/PA1 pt testcases/lens.txt output/lens.bmp nodebug
# bin/PA1 pt testcases/dao.txt output/daopt.bmp nodebug
# bin/PA1 pt testcases/smallpt.txt output/smallpt.bmp nodebug
# bin/PA1 npt testcases/vikingpt.txt output/vikingpt.bmp nodebug
# bin/PA1 testcases/scene01_basic.txt output/scene01.bmp
# bin/PA1 testcases/scene02_cube.txt output/scene02.bmp
# bin/PA1 testcases/scene03_sphere.txt output/scene03.bmp
# bin/PA1 testcases/scene04_axes.txt output/scene04.bmp
# bin/PA1 testcases/scene05_bunny_200.txt output/scene05.bmp
# bin/PA1 testcases/scene06_bunny_1k.txt output/scene06.bmp
# bin/PA1 testcases/scene07_shine.txt output/scene07.bmp
