import numpy as np
import sys
from matplotlib import pyplot as plt
from path import Path
from skimage import measure
import  os
import re
data_path = Path('/home/neousys/Project/volume_measurement/DataFromMobile/dinoSparseRing')
image_path = data_path/'IMAGE'
for i,img_file in enumerate(image_path.files('*.png')):
    index = re.findall(r"\d\d\d\d", img_file)
    dst = image_path/(str(int(index[0])) + ".png")
    print(img_file)
    os.rename(img_file, dst)