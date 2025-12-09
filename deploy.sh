#!/bin/bash

rsync -avz --delete --progress ./build ./install kt@f1ifs-desktop-1:~/repos/f1Tenth/
