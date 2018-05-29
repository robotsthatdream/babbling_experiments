#!\usr\bin\env python

import os
import sys

for iter_folder in os.listdir(sys.argv[1]) :
	if(iter_folder.split('_')[0] != "iteration") :
		continue
	iter_folder = sys.argv[1] + "/" + iter_folder
	for file in os.listdir(iter_folder) :
		if(file.split('_')[0] == "gmm") :
			file = iter_folder + "/" + file
			stream = open(file,'r')
			text = stream.read()
			stream.close()
			text = text.replace('-nan','0')
			text = text.replace('-inf','0')
			text = text.replace('nan','0')
			text = text.replace('inf','0')
			stream = open(file,'w')
			stream.write(text)
			stream.close()

