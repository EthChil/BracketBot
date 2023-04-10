import sys
import os.path
import time
import cv2


from slammer import fit_plane_model, uv_to_plane3d, plane3d_to_2d, project_on_plane, get_floor_points_mps, get_floor_points_kps, get_floor_contours



def main(vocab_path, settings_path, sequence_path):
    import orbslam2
    slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.MONOCULAR)
    slam.set_use_viewer(True)
    slam.initialize()

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
