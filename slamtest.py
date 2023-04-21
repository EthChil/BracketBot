import orbslam2
import cv2
import time
import numpy as np
import os

from slammer import fit_plane_model, uv_to_plane3d, plane3d_to_2d, project_on_plane, get_floor_points_mps, get_floor_points_kps, get_floor_contours


class OrbslamMono:
    def __init__(self, vocab_path, settings_path, sequence_path):
        self.vocab_path = vocab_path
        self.settings_path = settings_path
        self.sequence_path = sequence_path
        

    def load_images(self):
        raise NotImplementedError("Load images function not implemented")

    # def save_trajectory(self, filename):
    #     trajectory = self.slam.get_trajectory_points()
    #     with open(filename, 'w') as traj_file:
    #         traj_file.writelines('{time} {r00} {r01} {r02} {t0} {r10} {r11} {r12} {t1} {r20} {r21} {r22} {t2}\n'.format(
    #             time=repr(t),
    #             r00=repr(r00),
    #             r01=repr(r01),
    #             r02=repr(r02),
    #             t0=repr(t0),
    #             r10=repr(r10),
    #             r11=repr(r11),
    #             r12=repr(r12),
    #             t1=repr(t1),
    #             r20=repr(r20),
    #             r21=repr(r21),
    #             r22=repr(r22),
    #             t2=repr(t2)
            # ) for t, r00, r01, r02, t0, r10, r11, r12, t1, r20, r21, r22, t2 in trajectory)

    # def save_map_points(self):
    #     mps = self.slam.get_all_mappoints()
    #     mps = np.array(mps)
    #     np.savetxt('all_mappoints.txt', mps)

    #     floor_mps = self.slam.get_floor_mappoints()
    #     floor_mps = np.array(floor_mps)
    #     np.savetxt('floor_mappoints.txt', floor_mps)

    def run(self):
        rgb_filenames, timestamps = self.load_images()
        num_images = len(timestamps)

        slam = orbslam2.System(self.vocab_path, self.settings_path, orbslam2.Sensor.MONOCULAR)
        slam.set_use_viewer(True)
        slam.initialize()

        times_track = [0 for _ in range(num_images)]
        print('-----')
        print('Start processing sequence ...')
        print('Images in the sequence: {0}'.format(num_images))

        for idx in range(num_images):
            image = cv2.imread(os.path.join(self.sequence_path, rgb_filenames[idx]), cv2.IMREAD_UNCHANGED)
            tframe = timestamps[idx]

            if image is None:
                print("failed to load image at {0}".format(os.path.join(self.sequence_path, rgb_filenames[idx])))
                return 1

            t1 = time.time()
            
            print(image.shape, image.dtype, tframe)
            # print(os.path.join(sequence_path, rgb_filenames[idx]))
            # print(image, len(image), tframe)
            
            slam.process_image_mono(image, tframe)
            t2 = time.time()
        

            ttrack = t2 - t1
            times_track[idx] = ttrack

            t = 0
            if idx < num_images - 1:
                t = timestamps[idx + 1] - tframe
            elif idx > 0:
                t = tframe - timestamps[idx - 1]

            if ttrack < t:
                time.sleep(t - ttrack)

        save_trajectory(slam.get_trajectory_points(), 'trajectory.txt')

        slam.shutdown()

        times_track = sorted(times_track)
        total_time = sum(times_track)
        print('-----')
        print('median tracking time: {0}'.format(times_track[num_images // 2]))
        print('mean tracking time: {0}'.format(total_time / num_images))

        return 0


