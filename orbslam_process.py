#!/usr/bin/env python3

from sklearn.linear_model import LinearRegression
from sklearn.linear_model import RANSACRegressor

import sys
import os.path
import orbslam2
import time

from slammer import fit_plane_model, uv_to_plane3d, plane3d_to_2d, project_on_plane, get_floor_points_mps, get_floor_points_kps, get_floor_contours
import numpy as np
import cv2
    
def run_orbslam(vocab_path, settings_path, termination_event, orbslam_setup):
    slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.MONOCULAR)
    slam.set_use_viewer(True)
    slam.initialize()
    
    orbslam_setup.set()
    
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    if not video_capture.isOpened():
        print("Error: Unable to open camera")
        return
    
    start_time = time.time()
    times_track = []
    Rts = None
    driveable_area = []
    while time.time() < (start_time+6000) and not termination_event.is_set():
        ret_val, frame = video_capture.read()
        if not ret_val:
            continue

        t1 = time.time()
        slam.process_image_mono(frame, t1)
    
        mps = np.array(slam.get_tracked_mappoints())
        kps = np.array(slam.get_tracked_keypoints())

        # if mps.size > 0 and kps.size > 0:
        #     # floor_inds = get_floor_points_kps(kps)
        #     floor_inds = get_floor_points_mps(mps)
        #     slam.set_floor_mappoints(floor_inds.tolist())

        #     K = slam.get_intrinsics_matrix()
        #     Rt = slam.get_extrinsics_matrix()
        #     Rts = np.concatenate([Rts, Rt[np.newaxis]], axis=0) if Rts is not None else Rt[np.newaxis]
            
        #     # Fit ground plane
        #     floor_mps = np.array(slam.get_floor_mappoints())
        #     plane_coefs = fit_plane_model(floor_mps)
        #     slam.set_ground_plane_params(*plane_coefs)

        #     # Project floor contours
        #     floor_contours = get_floor_contours(frame)
        #     for contour in floor_contours:
        #         pts3d = uv_to_plane3d(contour, K, Rt, plane_coefs)
        #         # print("***********************************")
        #         # for pt, mp in zip(pts3d, mps[floor_inds]):
        #         #     print(pt, mp)
        #         # print("***********************************")
        #         # self.slam.add_floor_contour(pts3d)
        #         pts2d = plane3d_to_2d(pts3d, plane_coefs)
        #         driveable_area.append(pts2d)
        #     np.save('driveable_area.npy', np.stack(driveable_area))

        #     # Project camera path
        #     camera_path = project_on_plane(Rts[:,:,3], plane_coefs)
        #     camera_path = plane3d_to_2d(camera_path, plane_coefs)
        #     np.save('camera_path.npy', camera_path)

        t2 = time.time()
        ttrack = t2 - t1
        times_track.append(ttrack)
        

    save_trajectory(slam.get_trajectory_points(), 'trajectory.txt')
    slam.save_with_timestamps()
    slam.save_keyframe_trajectory()
    
    

    slam.shutdown()

    times_track = sorted(times_track)
    total_time = sum(times_track)
    print('-----')
    print('median tracking time: {0}'.format(times_track[num_images // 2]))
    print('mean tracking time: {0}'.format(total_time / num_images))

    return 0


def gstreamer_pipeline(sensor_id=0, capture_width=1280, capture_height=720, display_width=640, display_height=360, framerate=10, flip_method=0):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def save_trajectory(trajectory, filename):
    with open(filename, 'w') as traj_file:
        traj_file.writelines('{time} {r00} {r01} {r02} {t0} {r10} {r11} {r12} {t1} {r20} {r21} {r22} {t2}\n'.format(
            time=repr(stamp),
            r00=repr(r00),
            r01=repr(r01),
            r02=repr(r02),
            t0=repr(t0),
            r10=repr(r10),
            r11=repr(r11),
            r12=repr(r12),
            t1=repr(t1),
            r20=repr(r20),
            r21=repr(r21),
            r22=repr(r22),
            t2=repr(t2)
        ) for stamp, r00, r01, r02, t0, r10, r11, r12, t1, r20, r21, r22, t2 in trajectory)


if __name__ == '__main__':
    run_orbslam(sys.argv[1], sys.argv[2])
