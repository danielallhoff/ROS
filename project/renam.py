import os
import glob

NUMBER_OF_DIGITS = 5

for filename in glob.glob('images_frame_*.jpg'):
    index = filename[13:-4]
    os.rename(filename, 'images_frame_' + str(index).zfill(NUMBER_OF_DIGITS) + '.jpg')