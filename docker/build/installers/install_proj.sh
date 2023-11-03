#####################################################
# File Name: install_proj.sh
# Author: Leo
# mail: yli97@jmc.com.cn
# Created Time: 一  8/31 15:34:47 2020
#####################################################

#!/bin/bash

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

apt install sqlite3
tar xzf proj-4.9.3.tar.gz
pushd proj-4.9.3
./configure --prefix=/usr
make -j8
make install
popd

# Clean up.
rm -fr proj-4.9.3.tar.gz proj-4.9.3
