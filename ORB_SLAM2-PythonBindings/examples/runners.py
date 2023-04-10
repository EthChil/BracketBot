import orbslam2
import cv2
import time
import numpy as np

from slammer import fit_plane_model, uv_to_plane3d, plane3d_to_2d, project_on_plane, get_floor_points_mps, get_floor_points_kps, get_floor_contours


class OrbslamMono:
    def __init__(self, vocab_path, settings_path, sequence_path):
        self.vocab_path = vocab_path
        self.settings_path = settings_path
        self.sequence_path = sequence_path

        self.slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.MONOCULAR)

    def load_images(self):
        raise NotImplementedError("Load images function not implemented")

    def save_trajectory(self, filename):
        trajectory = self.slam.get_trajectory_points()
        with open(filename, 'w') as traj_file:
            traj_file.writelines('{time} {r00} {r01} {r02} {t0} {r10} {r11} {r12} {t1} {r20} {r21} {r22} {t2}\n'.format(
                time=repr(t),
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
            ) for t, r00, r01, r02, t0, r10, r11, r12, t1, r20, r21, r22, t2 in trajectory)

    def save_map_points(self):
        mps = self.slam.get_all_mappoints()
        mps = np.array(mps)
        np.savetxt('all_mappoints.txt', mps)

        floor_mps = self.slam.get_floor_mappoints()
        floor_mps = np.array(floor_mps)
        np.savetxt('floor_mappoints.txt', floor_mps)

    def run(self):
        import os
        if os.path.exists('camera_position.txt'):
            os.remove('camera_position.txt')
        image_filenames, timestamps = self.load_images()
        num_images = len(image_filenames)

        self.slam.set_use_viewer(True)
        self.slam.initialize()

        times_track = [0 for _ in range(num_images)]
        print('-----')
        print('Start processing sequence ...')
        print('Images in the sequence: {0}'.format(num_images))

        Rts = None
        driveable_area = []
        for idx in range(num_images):
            image = cv2.imread(image_filenames[idx], cv2.IMREAD_UNCHANGED)
            tframe = timestamps[idx]

            if image is None:
                print("failed to load image at {0}".format(image_filenames[idx]))
                return 1

            t1 = time.time()
            self.slam.process_image_mono(image, tframe)
            t2 = time.time()

            mps = np.array(self.slam.get_tracked_mappoints())
            kps = np.array(self.slam.get_tracked_keypoints())

            if mps.size > 0 and kps.size > 0:
                # floor_inds = get_floor_points_kps(kps)
                floor_inds = get_floor_points_mps(mps)
                self.slam.set_floor_mappoints(floor_inds.tolist())

                K = self.slam.get_intrinsics_matrix()
                Rt = self.slam.get_extrinsics_matrix()
                Rts = np.concatenate([Rts, Rt[np.newaxis]], axis=0) if Rts is not None else Rt[np.newaxis]

                # Fit ground plane
                floor_mps = np.array(self.slam.get_floor_mappoints())
                plane_model = fit_plane_model(floor_mps)
                self.slam.set_ground_plane_params(*plane_model.params)

                # Project floor contours
                floor_contours = get_floor_contours(image)
                for contour in floor_contours:
                    pts3d = uv_to_plane3d(contour, K, Rt, plane_model)
                    # print("***********************************")
                    # for pt, mp in zip(pts3d, mps[floor_inds]):
                    #     print(pt, mp)
                    # print("***********************************")
                    # self.slam.add_floor_contour(pts3d)
                    pts2d = plane3d_to_2d(pts3d, plane_model)
                    driveable_area.append(pts2d)
                np.save('driveable_area.npy', np.stack(driveable_area))

                # Project camera path
                camera_path = project_on_plane(Rts[:,:,3], plane_model)
                camera_path = plane3d_to_2d(camera_path, plane_model)
                np.save('camera_path.npy', camera_path)

                # if Rt is not None:
                #     Rts.append(np.concatenate([[tframe], Rt.flatten()]))
                #     # good = np.allclose(Rt.flatten(), np.array(Rt2[-1])[1:])
                #     # if not good:
                #     #     print('Bad Rt')
                #     #     print(Rt)
                # else:
                #     Rts.append(np.array([tframe, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]))

            ttrack = t2 - t1
            times_track[idx] = ttrack

            t = 0
            if idx < num_images - 1:
                t = timestamps[idx + 1] - tframe
            elif idx > 0:
                t = tframe - timestamps[idx - 1]

            if ttrack < t:
                time.sleep(t - ttrack)

        self.save_trajectory('trajectory.txt')
        np.savetxt('trajectory_unoptimized.txt', np.stack(Rts))
        self.save_map_points()

        self.slam.shutdown()

        times_track = sorted(times_track)
        total_time = sum(times_track)
        print('-----')
        print('median tracking time: {0}'.format(times_track[num_images // 2]))
        print('mean tracking time: {0}'.format(total_time / num_images))

        return 0

