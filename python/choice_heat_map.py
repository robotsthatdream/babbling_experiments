#!\usr\bin\env python

import os
import sys
import subprocess as sp


if len(sys.argv) != 2:
	print("Usage: arg1 : a folder that contain archives")
	sys.exit(1)

for folder in os.listdir(sys.argv[1]) :
	print(folder)
	sp.call(["roslaunch","dream_babbling","choice_heat_map.launch", \
		"load_exp:="+ sys.argv[1] + "/" + folder]) 