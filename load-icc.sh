#!/bin/bash

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

xcalib -d :0 "${CURRENT_DIR}/assets/icc/N156BGE-L41 #1 2016-11-24 04-15 2.2 M-S XYZLUT+MTX.icc"
