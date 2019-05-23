#!/usr/bin/python3
import sys
import time
import logging
import add_markers
import os
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# Indicate the openMVG binary directory
OPENMVG_SFM_BIN = "/home/neousys/Software/openMVG_Build_tag/Linux-x86_64-RELEASE"

# Indicate the openMVG camera sensor width directory
CAMERA_SENSOR_WIDTH_DIRECTORY = "/home/neousys/Software/openMVG/src/software/SfM" + "/../../openMVG/exif/sensor_width_database"

input_dir = ''
output_dir = ''
matches_dir = ''
reconstruction_dir = ''
camera_file_params = os.path.join(CAMERA_SENSOR_WIDTH_DIRECTORY, "sensor_width_camera_database.txt")

class AddImageEventHandler(FileSystemEventHandler):
    def __init__(self):
        ## image counter in every measure
        self.image_counter = 0

    def on_created(self, event):
        super(AddImageEventHandler, self).on_created(event)

        what = 'directory' if event.is_directory else 'file'
        logging.info("Created %s: %s", what, event.src_path)
        if (what == 'file'): 
            self.image_counter = self.image_counter + 1 
            logging.info("there are %d image" % self.image_counter)

class SfmEventHandler(FileSystemEventHandler):
    """Logs all the events captured."""
    def __init__(self):
        ## measure counter
        self.measure_counter = 0
        self.image_observer = Observer()
        self.image_observer.start()
        ## 
    def on_created(self, event):
        super(SfmEventHandler, self).on_created(event)

        what = 'directory' if event.is_directory else 'file'
        logging.info("Created %s: %s", what, event.src_path)

        ## if creating a directory "image" means starting a new measurement
        if (what == 'directory'): 
            self.measure_counter = self.measure_counter + 1
            image_event_handler = AddImageEventHandler()
            ## assign sfm directory
            input_dir = os.path.join(event.src_path, 'image')
            if not os.path.exists(input_dir):
                os.makedirs(input_dir)
            output_dir = event.src_path
            matches_dir = os.path.join(output_dir, "matches")
            reconstruction_dir = os.path.join(output_dir, "reconstruction_sequential")
            self.image_observer.unschedule_all()
            self.image_observer.schedule(image_event_handler, input_dir, recursive=False)
            
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')
    path = sys.argv[1] if len(sys.argv) > 1 else '.'
    event_handler = SfmEventHandler()
    observer = Observer()
    observer.schedule(event_handler, path, recursive=False)
    observer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()