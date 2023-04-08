#!/usr/bin/env python3
import sys
import os.path
import orbslam2
import time
import cv2

def gstreamer_pipeline(sensor_id=0, capture_width=1920, capture_height=1080, display_width=960, display_height=540, framerate=30, flip_method=0):
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


def main(vocab_path, settings_path):
    
    start_time = time.time()

    slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.MONOCULAR)
    slam.set_use_viewer(True)
    slam.initialize()

    # times_track = [0 for _ in range(num_images)]
    print('-----')
    print('Start processing sequence ...')
    # print('Images in the sequence: {0}'.format(num_images))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        while time.time() < (start_time+10000):
            loop_start_time = time.time()
            
            ret_val, frame = video_capture.read()
            if ret_val:
                # cv2.imwrite(filename, frame)
                t1 = time.time()
                resized_frame = cv2.resize(frame, (960, 540))
                slam.process_image_mono(resized_frame, t1)
                
            loop_end_time = time.time()
            elapsed_time = loop_end_time - loop_start_time

            if elapsed_time < 0.05:
                time.sleep(0.05 - elapsed_time)
    else:
        print("Error: Unable to open camera")
            
        


    save_trajectory(slam.get_trajectory_points(), 'trajectory.txt')

    slam.shutdown()

    # times_track = sorted(times_track)
    # total_time = sum(times_track)
    print('-----')
    # print('median tracking time: {0}'.format(times_track[num_images // 2]))
    # print('mean tracking time: {0}'.format(total_time / num_images))

    return 0


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
    if len(sys.argv) != 5:
        print('Usage: ./orbslam_mono_tum path_to_vocabulary path_to_settings')
    main(sys.argv[1], sys.argv[2])
