#Helper functions for RomiTrackerV2.py
import cv2
import numpy as np
import sys
import time
from cv2 import aruco
import json
import struct

PACKETSIZE = 163 #bytes for 10 Romis

#Define a class to hold pixel to world transformation parameters
class Transform:
    def __init__(self, R, tvec, Knew):
        #Prepare for some gnarly algebra to convert pixel coordinates to world coordinates
        #Build world to camera transform (2.5.14)
        R_t = np.transpose(R)  # Transpose of rotation matrix
        tR_t = -R_t @ tvec.reshape(3, 1)  # Compute -R^T * tvec

        w_T_c = np.vstack((np.hstack((R_t, tR_t)), np.array([[0, 0, 0, 1]])))  # Combine R and tvec into a 3x4 matrix

        #Define variables to make code more readable
        self.r11 = w_T_c[0, 0]
        self.r12 = w_T_c[0, 1]
        self.r13 = w_T_c[0, 2]
        self.t1 = w_T_c[0, 3]
        self.r21 = w_T_c[1, 0]
        self.r22 = w_T_c[1, 1]
        self.r23 = w_T_c[1, 2]
        self.t2 = w_T_c[1, 3]
        self.r31 = w_T_c[2, 0]
        self.r32 = w_T_c[2, 1]
        self.r33 = w_T_c[2, 2]
        self.t3 = w_T_c[2, 3]

        self.cx = Knew[0, 2]
        self.cy = Knew[1, 2]
        self.fx = Knew[0, 0]
        self.fy = Knew[1, 1]

    def pixel_to_world(self, pixel_coords, Zw):
        #Inputs: Pixel coordinates (pixel_coords = [u, v]), Zw is the world Z coordinate (height)
        #Returns: Xw, Yw are the world coordinates corresponding to the pixel coordinates at height Zw

        u = pixel_coords[:, 0]
        v = pixel_coords[:, 1]

        #Normalize pixel coordinates
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        #Find Zc
        Zc = (-Zw - self.t3) / (self.r31*x + self.r32*y + self.r33)
        #Find world coordinates
        Xw = self.r11*x*Zc + self.r12*y*Zc + self.r13*Zc + self.t1
        Yw = self.r21*x*Zc + self.r22*y*Zc + self.r23*Zc + self.t2
        return np.column_stack((Xw, Yw))

#Class that implements the entire computer vision algorithm
class RomiCVAlgorithm:
    def __init__(self, img):
        #Some configuration parameters
        self.LocatingMarkers = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10] #Markers used to locate the camera

        num_markers = 20 #Max number of Romis we expect to detect

        #Known marker positions in mm (x, y) for the 11 locating markers
        MarkerCenters_mm = np.array([
            [  70.0, 847.2],  # ID 0
            [  70.0,  67.2],  # ID 1
            [ 380.0, 457.2],  # ID 2
            [ 690.0, 847.2],  # ID 3
            [ 690.0,  67.2],  # ID 4
            [1000.0, 457.2],  # ID 5
            [1310.0, 847.2],  # ID 6
            [1310.0,  67.2],  # ID 7
            [1620.0, 457.2],  # ID 8
            [1930.0, 847.2],  # ID 9
            [1930.0,  67.2],  # ID 10
        ])

        MarkerWidth_mm = 100.0 #Width of the markers in mm

        #Read K and D from JSON files
        with open('K.json', 'r') as f:
            data = json.load(f)
        self.K = np.array(data)

        with open('D.json', 'r') as f:
            data = json.load(f)
        self.D = np.array(data)

        print(f'Loaded K and D from JSON files.')

        #img = cv2.resize(original_img, None, fx=scaledown_factor, fy=scaledown_factor, interpolation=cv2.INTER_AREA)
        h, w = img.shape[:2]
        dim = (w, h)
        print(f'Captured image with dimensions: {dim}')
        cv2.imwrite('./debug_pics/0original_image.jpg', img)


        if dim != (4608, 2592): #If dimensions not as expected, we need to scale K
            scale = dim[0] / 4608
            self.K = self.K * scale
            print(f"Scaled K by {scale}")

        #Init map for undistortion
        R = np.eye(3)
        #New intrinsic matrix
        self.Knew = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.K, self.D, dim, R, balance=0, new_size=dim)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, R, self.Knew, dim, cv2.CV_16SC2)
        print(f'Initialized undistortion maps.')

        #Preallocate arrays to store latest Pos data
        self.pose_data = np.zeros((num_markers, 4)) #ID, center x, center y, heading


        #Setup ArUco detector
        #Setup aruco dictionary
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        #Config Aruco detector parameters
        parameters = cv2.aruco.DetectorParameters() #Get default parameters
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX #Refine the marker corners more!!


        # Create the ArUco detector
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        #Flatten the Image
        flattened = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        cv2.imwrite('./debug_pics/1post-defisheye.jpg', flattened)

        #Detect the markers
        corners, ids, _ = self.detector.detectMarkers(flattened)

        #Check and make sure we found the known markers
        if not all(marker_id in ids for marker_id in self.LocatingMarkers):
            raise ValueError("Not all required markers were detected in the image.")

        #Allocate array to store pixel coordinates of corners for 4 markers
        marker_pixel_coords = np.zeros((4 * len(self.LocatingMarkers), 2), dtype=np.float32)

        #Build MarkerPixelCoords array so that markers are in the same order as MarkerPoses_mm (0 -> 11)
        for marker_id in range(len(self.LocatingMarkers)):  # Assuming marker IDs are 1 to 11
            #Get the appropriate index for this marker ID
            idx = np.where(ids == marker_id)[0][0]

            #Corners is a tuple of arrays which contains exactly one element: an array of four arrays each with a corner coordinate
            #So we have to double index to get to the actual coordinates
            marker_pixel_coords[4*(marker_id) + 0] = corners[idx][0][0]  # Top-left corner
            marker_pixel_coords[4*(marker_id) + 1] = corners[idx][0][1]  # Top-right corner
            marker_pixel_coords[4*(marker_id) + 2] = corners[idx][0][2]  # Bottom-right corner
            marker_pixel_coords[4*(marker_id) + 3] = corners[idx][0][3]  # Bottom-left corner

        marker_pixel_coords = np.ascontiguousarray(marker_pixel_coords, dtype=np.float32)  # Convert to numpy Contigous array

        #Now, so we only look at the game track going forwad, crop the image based on the center points of the 4 markers
        self.x1 = min(int(marker_pixel_coords[4*(1)+0,0]), int(marker_pixel_coords[4*(0)+0,0])) #Either Marker 1, corner 1 or Marker 0, corner 0, which ever is more left
        self.y1 = min(int(marker_pixel_coords[4*(1)+0,1]), int(marker_pixel_coords[4*(10)+2,1])) #Either Marker 1 conrer 1 or Marker 10 corner 2 y-pos, which ever is higher
        self.x2 = max(int(marker_pixel_coords[4*(10)+2,0]), int(marker_pixel_coords[4*(9)+3,0])) #Either Marker 10 corner 2 or Marker 9 corner 3 x-pos, which ever is more right
        self.y2 = max(int(marker_pixel_coords[4*(0)+0,1]), int(marker_pixel_coords[4*(9)+3,1])) #Either Marker 0 corner 0 or Marker 9 corner 3 y-pos, which ever is lower
        print(f"Cropping image to x:[{self.x1}:{self.x2}], y:[{self.y1}:{self.y2}]")

        img_cropped = flattened[self.y1:self.y2, self.x1:self.x2]
        cv2.imwrite('./debug_pics/2cropped.jpg', img_cropped)

        # Generate MarkerPoses_mm for all detected points (marker corners) (each marker has 4 corners: 0..3)
        MarkerPoses_mm = []
        for i in range(len(MarkerCenters_mm)):
            cx, cy = MarkerCenters_mm[i]
            half = MarkerWidth_mm / 2.0
            MarkerPoses_mm.extend([
                [cx - half, cy + half, 0],  # Corner 0
                [cx - half, cy - half, 0],  # Corner 1
                [cx + half, cy - half, 0],  # Corner 2
                [cx + half, cy + half, 0],  # Corner 3
            ])
        MarkerPoses_mm = np.ascontiguousarray(MarkerPoses_mm, dtype=np.float32)  # Convert to numpy array

        #Compute rotation and translation vectors using solvePnP (Camera pose estimation)
        success, rvec, tvec, inliers = cv2.solvePnPRansac(MarkerPoses_mm, marker_pixel_coords, self.Knew, None, iterationsCount=1000, flags=cv2.SOLVEPNP_ITERATIVE)
        print(f"RANSAC rejected {len(marker_pixel_coords) - len(inliers)} out of {len(marker_pixel_coords)} points as outliers.")
        #Convert rotation vector to rotation matrix
        R, _ = cv2.Rodrigues(rvec)
        #Instantiate the Transform class with the computed R, tvec, and Knew
        self.transform = Transform(R, tvec, self.Knew)
        print(f"Translation vector: {tvec.ravel()}")
        print(f"Rotation matrix:\n{R}")

    #@profile
    def track_Romi(self, img):
        #Detects markers, computes world coordinates, returns pose array and number of detected markers

        #Flatten the image
        flattened = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        #Crop the image
        img_cropped = flattened[self.y1:self.y2, self.x1:self.x2]
        # Detect the markers
        corners, ids, _ = self.detector.detectMarkers(img_cropped)

        if ids is not None:    #Display results
            pose_idx = 0    #for iterating through pose_data array
            #Unpack array of tag corner coordinate
            for idx, tag in enumerate(corners):

                #Get marker ID
                id = ids[idx][0]
                if id in self.LocatingMarkers:
                    continue  # Skip processing for locating markers

                self.pose_data[pose_idx][0] = ids[idx][0]

                #Shift coordinates to cropped image coordinates
                tag[0][:, 0] += self.x1  # Shift x-coordinates
                tag[0][:, 1] += self.y1  # Shift y-coordinates

                #Transform corners to world coords
                world_coords = self.transform.pixel_to_world(tag[0], 130)


                #Unpack corner coordinates
                x_tl, y_tl = world_coords[0]  # Top-left corner
                x_tr, y_tr = world_coords[1]  # Top-right corner
                x_br, y_br = world_coords[2]  # Bottom-right corner
                x_bl, y_bl = world_coords[3]  # Bottom-left corner

                #Do some averaging b/c it's not pefectly rectangular
                x_l = (x_tl + x_bl) / 2.0
                x_r = (x_tr + x_br) / 2.0
                y_t = (y_tl + y_tr) / 2.0
                y_b = (y_bl + y_br) / 2.0

                #Calculate tag center point and translate to cropped image coords
                cX = (x_l + x_r) / 2.0
                cY = (y_t + y_b) / 2.0
                #Store center points
                self.pose_data[pose_idx][1] = cX
                self.pose_data[pose_idx][2] = cY

                #Calcuate heading angle
                #Look at vector from bottom-right to bottom-left
                deltaX = x_bl - x_br
                deltaY = y_bl - y_br
                angle_rad = np.arctan2(deltaY, deltaX)
                angle_deg = np.degrees(angle_rad)
                #Store heading angle
                self.pose_data[pose_idx][3] = angle_deg

                pose_idx += 1
            return self.pose_data[:pose_idx, :]  # Return the useful pose data
        else:
            return None  # No markers detected



#Function to print latest pos data
def display_pose_table(pose_data: np.ndarray, t_total, t_frame_to_cbck, n: int) -> None:

    #Sort by Marker ID
    sorted_idxs = np.argsort(pose_data[0:n,0])

    # Fixed column widths so table never shifts
    W_ID = 9
    W_POS = 21
    W_HEAD = 8

    #Helper Functions
    def hline():
        return f"+{'-'*(W_ID+2)}+{'-'*(W_POS+2)}+{'-'*(W_HEAD+2)}+"

    def row(a, b, c):
        return f"| {a:<{W_ID}} | {b:<{W_POS}} | {c:<{W_HEAD}} |"

    lines = [
        hline(),
        row("Marker ID", "Position", "Heading"),
        hline(),
    ]

    for idx in sorted_idxs:
        id, x, y, heading = pose_data[idx]
        lines.append(
            row(
                str(int(id)),
                f"({x:07.3f}, {y:07.3f})",
                f"{heading:07.3f}",
            )
        )

    lines.append(hline())

    # Clear screen and redraw
    sys.stdout.write("\x1b[2J\x1b[H")
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.write(f"Total Elapsed Time: {t_total:.4f} seconds\n")
    sys.stdout.write(f"Frame to Callback Time: {t_frame_to_cbck:.4f} seconds\n")
    sys.stdout.flush()

#Function to build packet as a byte array from a np array
def build_packet(data: np.ndarray, buffer: bytearray, n: int): #Pose data for 10 Romis, preallocated buffer, and length of data (num of tags detected)
    #Add Heading
    struct.pack_into('<BB', buffer, 0, 0x5A, 0x5A) #Header
    struct.pack_into('<B', buffer, 2, n) #number of Romis detected
    #Add data for detcted Romis
    for i in range(n):
        struct.pack_into('<ffff', buffer, 3+i*16, data[i,0], data[i,1], data[i,2], data[i,3]) #Appropriate offset is after heading and number of romi datas inputted
    #Add zeros everywhere else
    if n < 10:
        start_offset = 3 + n*16
        num_zeros = PACKETSIZE-start_offset
        buffer[start_offset : start_offset + num_zeros] = b'\x00' * num_zeros
