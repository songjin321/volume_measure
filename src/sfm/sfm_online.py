#!/usr/bin/python3
import sys
import time
import logging
import add_markers
from measurement import Measurement
import os
import subprocess
import multiprocessing
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from open3d import *
# Indicate the openMVG binary directory
OPENMVG_SFM_BIN = "/home/neousys/Software/openMVG_Build_tag/Linux-x86_64-RELEASE"

# Indicate the openMVG camera sensor width directory
CAMERA_SENSOR_WIDTH_DIRECTORY = "/home/neousys/Software/openMVG/src/software/SfM" + "/../../openMVG/exif/sensor_width_database"

input_dir = ''
output_dir = ''
matches_dir = ''
reconstruction_dir = ''
camera_file_params = os.path.join(CAMERA_SENSOR_WIDTH_DIRECTORY, "sensor_width_camera_database.txt")

# for visualizer
"""
line_set = LineSet() # object box
pcd = PointCloud()   # reconstruted point cloud
length = width = height = []
vis = Visualizer()
vis.create_window()
vis.add_geometry(pcd)
vis.add_geometry(line_set)
vis.update_geometry()
vis.poll_events()
vis.update_renderer()
"""

def renconstruct_measure():
    print("*********start a measurement****************")
    print ("3. Compute matches")
    pMatches = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeMatches"),  "-i", matches_dir+"/sfm_data.json", "-o", matches_dir] )
    pMatches.wait()

    print ("4. Do Sequential/Incremental reconstruction")
    pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_IncrementalSfM"),  "-i", matches_dir+"/sfm_data.json", "-m", matches_dir, "-o", reconstruction_dir] )
    pRecons.wait()

    print ("5. Convert bin to json")
    pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ConvertSfM_DataFormat"),  "-i", reconstruction_dir+"/sfm_data.bin", "-o", reconstruction_dir+"/sfm_data.json"] )
    pRecons.wait()

    print ("6. Generate sfm_data_markers.json using markers")
    add_markers.add_aruco_markers(reconstruction_dir+"/sfm_data.json")

    print ("7. Generate sfm_data_aligned.bin with ControlPointsRegistration")
    pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ControlPointsRegistration"),  "-i", reconstruction_dir+"/sfm_data_markers.json", "-o", reconstruction_dir+"/sfm_data_aligned.bin"] )
    pRecons.wait()

    # compute final valid structure from the known camera poses
    print ("8. Structure from Known Poses (robust triangulation)")
    pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeStructureFromKnownPoses"),  "-i", reconstruction_dir+"/sfm_data_aligned.bin", "-m", matches_dir, "-f", os.path.join(matches_dir, "matches.f.bin"), "-o", os.path.join(reconstruction_dir,"robust_aligned.bin")] )
    pRecons.wait()

    pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeSfM_DataColor"),  "-i", reconstruction_dir+"/robust_aligned.bin", "-o", os.path.join(reconstruction_dir,"robust_colorized.ply")] )
    pRecons.wait()    

    print ("9. Measure volume")
    mes_instance = Measurement()
    mes_instance.measure(os.path.join(reconstruction_dir, 'robust_aligned.ply'))

    # show result
    """
    global pcd, line_set, length, width, height
    pcd = mes_instance.pcd
    line_set = mes_instance.line_set
    [length, width, height] = mes_instance.result
    """
    
class AddImageEventHandler(FileSystemEventHandler):
    def __init__(self):
        ## image counter in every measure
        self.image_counter = 1
        self.counter_timeline = 10
        self.p = None
    def on_created(self, event):
        super(AddImageEventHandler, self).on_created(event)

        what = 'directory' if event.is_directory else 'file'
        logging.info("Created %s: %s, start a object measurement", what, event.src_path)
        if (what == 'file'): 
            self.image_counter = self.image_counter + 1 
            ## for each image extract sift feature
            print ("1. Intrinsics analysis")
            print(input_dir)
            pIntrisics = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_SfMInit_ImageListing"),  "-i", input_dir, "-o", matches_dir, "-d", camera_file_params, "-k", "1583.57344776699; 0.; 500.842288940575; 0.; 1584.03592624341; 944.606239259718; 0.; 0.; 1."] )
            pIntrisics.wait()

            print ("2. Compute features")
            pFeatures = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeFeatures"),  "-i", matches_dir+"/sfm_data.json", "-o", matches_dir, "-m", "SIFT"] )
            pFeatures.wait()

            print ("\033[93m Extracted image %d features\033[0m" % self.image_counter)
            # Take measurement every 5 pictures
            if self.image_counter >= self.counter_timeline:
                ## use a process to reconstruction and calculate volume
                if (self.p is None) or (not self.p.is_alive()):
                    self.p = multiprocessing.Process(target=renconstruct_measure)
                    self.p.start()
                    self.counter_timeline = self.image_counter + 5

class SfmEventHandler(FileSystemEventHandler):
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
            global input_dir, output_dir, matches_dir, reconstruction_dir
            input_dir = os.path.join(event.src_path, 'image')
            output_dir = event.src_path
            matches_dir = os.path.join(output_dir, "matches")
            reconstruction_dir = os.path.join(output_dir, "reconstruction_sequential")

            # Create the input match reconstruction directory if not present
            if not os.path.exists(input_dir):
                os.makedirs(input_dir)
            if not os.path.exists(matches_dir):
                os.mkdir(matches_dir)
            if not os.path.exists(reconstruction_dir):
                os.mkdir(reconstruction_dir)

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