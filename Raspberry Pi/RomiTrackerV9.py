#Uses Version 3 - all the latest bells and whistles
#Rollback from Version 4 - image processing done in callback rather than another thread b/c this is faster.
#Big new implementation here is sending stuff over SPI (New Thread)
#Initialization code
import cv2
from cv2 import aruco
import json
import numpy as np
import sys
from picamera2 import Picamera2
from picamera2.request import MappedArray
import time
from RomiTrackerV9_helper import RomiCVAlgorithm, display_pose_table, build_packet
import threading
import queue
import spidev
import numpy as np
import struct
from gpiozero import LED
import csv


#@profile
def on_frame(request):
    t_start = request.get_metadata().get("SensorTimestamp", None) # Nanoseconds timestamp provided by libcamera / sensor pipeline
    t_cbck = time.clock_gettime_ns(time.CLOCK_BOOTTIME) # Nanoseconds timestamp when callback is called
    with MappedArray(request, "main") as m:
        yuv = m.array
        gray_image = yuv[:frame_h, :frame_w]   # Y plane only (no color)
        pose_data = tracker.track_Romi(gray_image) #Preform marker detection and coordinate transformation to get pose data
        if pose_data is not None:
            SPI_data_queue.put(pose_data) #Send pose data to SPI thread
            t_end = time.clock_gettime_ns(time.CLOCK_BOOTTIME) # Nanoseconds timestamp when callback is done
            t_total = t_end - t_start
            t_frame_to_cbck = t_cbck - t_start
            print_data_queue.put([pose_data, t_total, t_frame_to_cbck]) #Send pose data to printer thread
            print_flag.set()  # Signal printer thread
            write_data_queue.put([t_total, t_frame_to_cbck]) #Send timing data to writer thread
            write_flag.set()  # Signal writer thread
        else:
            print("No markers detected in this frame!")

def SPI_thread_fun(SPI_data_queue, spi, buff):
    print("SPI thread started")
    while True: #Block until we get pose data
        pose_data = SPI_data_queue.get()
        build_packet(pose_data, buff, len(pose_data)) #Build packet from pose data
        payload = buff.ljust(256, b"\x00")             # pad to 256 bytes
        profile_pin.toggle() #Toggle profile pin
        spi.xfer2(list(payload))                     # rx contains MISO bytes


def printer_thread_fun(pose_data_queue, print_flag):
    print("Printer thread started.")
    while True:
        print_flag.wait()
        pose_data, t_total, t_frame_to_cbck = pose_data_queue.get()
        display_pose_table(pose_data, t_total/1e9, t_frame_to_cbck/1e9, len(pose_data))
        print_flag.clear()

def writer_thread_fun(pose_data_queue, write_flag):
    print("Writer thread started.")
    while True:
        write_flag.wait()
        with open('timing_log.csv', mode='a', newline='') as file:
            writer = csv.writer(file)
            t_total, t_frame_to_cbck = pose_data_queue.get()
            writer.writerow([t_total/1e9, t_frame_to_cbck/1e9])
        write_flag.clear()



#Init Camera
picam2 = Picamera2()
#frame_w, frame_h = 768, 432  #Known good frame size
frame_w, frame_h = 2304, 1296
config = picam2.create_video_configuration(
    main={"size": (frame_w, frame_h), "format": "YUV420"},  # fast grayscale via Y plane
    sensor={"output_size": (2304, 1296), "bit_depth": 10}, #This is the optimal sensor config
    buffer_count=1,
    queue=False,
    controls = {
    "AeEnable": True,          # let AE run
    "ExposureTime": 5000,      # 5 ms cap (µs)
    "NoiseReductionMode": 0,
    "FrameDurationLimits": (16666, 16666),  # ~60 fps
    #NOTE: Frame (framerate) is limited to 17849 (54.2 or something fps) by the sensor config!
}
)

picam2.configure(config)
picam2.start()
print('Initializing Camera...')
time.sleep(3)  # allow exposure + autofocus to settle

#Capture a test image to init maps and homography
original_img = picam2.capture_array()

img = original_img[:frame_h, :frame_w] #Extract Y plane (grayscale) for processing

#Init CV algorithm
tracker = RomiCVAlgorithm(img)

#Init SPI and profiling pin
profile_pin = LED(21)

spi = spidev.SpiDev()
spi.open_path("/dev/spidev0.0")

# SPI Settings
spi.max_speed_hz = 6_000_000  # 10 MHz (ESP32 max)
spi.mode = 0b01                                 #Set SPI to mode 1 (must match ESP32's settings!)

#Preallocate buffer to pack packet data into.
PACKETSIZE = 163 #bytes for 10 Romis
buff = bytearray(PACKETSIZE) #163 bytes for 10 Romis

#Set up SPI thread
SPI_data_queue = queue.Queue()
SPI_thread = threading.Thread(target=SPI_thread_fun, args=(SPI_data_queue, spi, buff), daemon=True)
SPI_thread.start()

#Set up printer thread
print_data_queue = queue.Queue()
print_flag = threading.Event()
print_flag.clear()
printer_thread = threading.Thread(target=printer_thread_fun, args=(print_data_queue, print_flag), daemon=True)
printer_thread.start()

#Set up writer thread
write_data_queue = queue.Queue()
write_flag = threading.Event()
write_flag.clear()
#writer_thread = threading.Thread(target=writer_thread_fun, args=(write_data_queue, write_flag), daemon=True)
#writer_thread.start()

picam2.stop()
print("Starting main loop...")
picam2.pre_callback = on_frame
picam2.start()

try:
    while True:
        time.sleep(1)  #Keep main thread alive while camera callback, print, and SPI threads run in the background
except KeyboardInterrupt:
    picam2.stop()
    printer_thread.join()
    SPI_thread.join()
    spi.close()