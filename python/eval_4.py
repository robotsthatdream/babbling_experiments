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
"normalHistLarge":27, \
"fpfh":33, \
"colorHSVNormal":6, \
"colorRGBNormal":6, \
"colorLab":6, \
"colorLabHist":15, \
"colorLabNormalHist":30, \
"neighMeanFPFH" :33, \
"merge":0}

for folder in os.listdir(sys.argv[1]) :
	print(folder)
	
	sp.call(["roslaunch","dream_babbling","classifier_eval.launch", \
		"archive_folder:="+ sys.argv[1] + folder, \
		"method:=gmm", \
		"modality:=neighMeanFPFH", \
		"dimension:="+str(dimensions["neighMeanFPFH"]), \
		"number_of_iteration:=100", \
		"output_file:=classifier_eval_neighMeanFPFH.yml"]) 

for folder in os.listdir(sys.argv[1]) :
	print(folder)
	
	sp.call(["roslaunch","dream_babbling","classifier_eval.launch", \
		"archive_folder:="+ sys.argv[1] + folder, \
		"method:=gmm", \
		"modality:=colorLabHist", \
		"dimension:="+str(dimensions["colorLabHist"]), \
		"number_of_iteration:=100", \
		"output_file:=classifier_eval_colorLabHist.yml"]) 



