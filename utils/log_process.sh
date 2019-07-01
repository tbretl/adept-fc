#
# Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
#
# Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
#

#!/bin/bash

#read cpu usage of each process:
rm process_use.dat
pidstat -p ALL 1  > process_use.dat
