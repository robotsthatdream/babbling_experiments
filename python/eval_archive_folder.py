#!\usr\bin\env python

import os
import sys
import subprocess as sp

dimensions = \
{"merged":63, \
"colorNormalHist":45, \
"colorHSV":3, \
"colorRGB":3, \
"colorH":5, \
"colorS":5, \
"colorV":5, \
"colorHist":30, \
"normal":3, \
"normalX":5, \
"normalY":5, \
"normalZ":5, \
"normalHist":15, \
"fpfh":33, \
"colorHSVNormal":6, \
"colorRGBNormal":6, \
"colorLab":6, \
"colorLabHist":15, \
"colorLabNormalHist":30, \
"meanFPFHLabHist":48, \
"merge":0}


if len(sys.argv) < 6:
	print("Usage: arg1 : a folder that contain archives")
	print("arg2 : method for generating saliency map")
	print("arg3 : modality")
	print("arg4 : number of iteration")
	print("arg5 : output file")
	sys.exit(1)

for folder in os.listdir(sys.argv[1]) :
	print(folder)
	
	sp.call(["roslaunch","dream_babbling","classifier_eval.launch", \
		"archive_folder:="+ sys.argv[1] + folder, \
		"method:=" + sys.argv[2], \
		"modality:="+sys.argv[3], \
		"dimension:="+str(dimensions[sys.argv[3]]), \
		"number_of_iteration:=" + sys.argv[4], \
		"output_file:=" + sys.argv[5]]) 