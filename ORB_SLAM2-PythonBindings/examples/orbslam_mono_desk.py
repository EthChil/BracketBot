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

# def plot_floor_pts(floor_mps, plane_model):
#     # Import outside breaks ORBSLAM
#     import matplotlib as plt
#     fig = plt.figure()
#     ax = fig.add_subplot(projection='3d')

#     n = 20
#     x, y, z = floor_mps.T
#     ax.scatter(x, y, z, s=0.01, c='r')

#     xx = np.linspace(x.min(), x.max(), n)
#     yy = np.linspace(y.min(), y.max(), n)
#     X, Y = np.meshgrid(xx, yy)
#     Z = c0*X.flatten() + c1*Y.flatten() + c2
#     Z = Z.reshape(X.shape)
#     ax.plot_surface(X, Y, Z, shade=False)

#     ax.set_xlabel('X Label')
#     ax.set_ylabel('Y Label')
#     ax.set_zlabel('Z Label')

#     plt.show()


# def get_floor_points_kps(kps):
#     if len(kps) == 0:
#         return []

#     floor_inds = []
#     for i, (kpx, kpy) in enumerate(kps):
#         kpx = int(kpx)
#         kpy = int(kpy)
#         if 400 < kpx < 700 and kpy > 150:
#             floor_inds.append(i)

#     return floor_inds


# def get_floor_points_mps(mps):
#     if len(mps) == 0:
#         return []

#     zs = mps[:,1]
#     z_top10 = np.percentile(zs, 90)
#     floor_inds = np.argwhere(zs > z_top10)

#     return floor_inds.squeeze()


# def fit_plane_model(pts):
#     lin_reg = LinearRegression(fit_intercept=True)
#     ransac = RANSACRegressor(base_estimator=lin_reg,
#                              min_samples=10,
#                              residual_threshold=0.01,
#                              max_trials=1000)
#     xy = pts[:,:2]
#     z = pts[:,2]
#     ransac.fit(X, y)
    
#     coefs = ransac.estimator_.coef_ + [ransac.estimator_.intercept_]
#     return coefs


# def uv_to_plane3d(pts2d, K, Rt, plane_coefs):
#     """
#     :param pts2d: points to project (u, v)
#     :param K: Camera intrinsics matrix
#     :param Rt: Camera extrinsics matrix
#     :param plane_model: Plane3DModel object, output of fit_plane_model()
#     :return: 3D points representing projection of pts2d onto the floor plane in the 3D map space
#     """

#     pts3d = []

#     c0, c1, c2 = plane_coefs
#     fx = K[0, 0]
#     fy = K[1, 1]
#     cx = K[0, 2]
#     cy = K[1, 2]
#     R = Rt[:,:3]
#     t = Rt[:,3]

#     # Ax = b
#     A = np.array([[-1, 0, 0],[0, -1, 0],[-c0, -c1, 0]])
#     b = np.array([0, 0, c2]).T
#     for (u, v) in pts2d:
#         x_ = (u-cx)/fx
#         y_ = (v-cy)/fy
#         P = R.T @ np.array([x_, y_, 1]).T
#         Q = R.T @ t

#         A[:,2] = P
#         assert np.linalg.det(A) != 0

#         X, Y, z = np.linalg.inv(A) @ (Q+b)
#         Z = c0*X + c1*Y + c2
#         pt3d = np.array([X, Y, Z])
#         # Check that z is correct
#         # assert np.allclose(z*P - Q, pt3d, rtol=1e-6)

#         # Check that the projection is correct
#         screen_pt = K @ Rt @ np.append(pt3d, 1)
#         uv_proj = screen_pt[:2]/screen_pt[2]
#         # assert np.allclose(uv_proj, [u, v], rtol=1e-6)

#         pts3d.append(pt3d)

#     return np.stack(pts3d)


# def plane3d_to_2d(pts3d, plane_coefs):
#     """
#     :param pts3d: points to project (x, y, z)
#     :param plane_model: Plane3DModel object, output of fit_plane_model()
#     :return: 2D points representing projection of pts3d onto the ground plane
#     """
#     c0, c1, c2 = plane_coefs
#     origin = np.array([0, 0, c2])

#     # Forward, right and normal direction vectors for the ground plane
#     fwd = np.array([0, 1/c1, c2+1]) - origin
#     fwd /= np.linalg.norm(fwd)
#     right = np.array([1, (c2-c0)/c1, c2]) - origin
#     right /= np.linalg.norm(right)

#     d_right = np.dot(pts3d - origin, right)
#     d_fwd = np.dot(pts3d - origin, fwd)
#     pts2d = np.stack([d_right, d_fwd], axis=1)
#     return pts2d


# def project_on_plane(pts3d, plane_coefs):
#     c0, c1, c2 = plane_coefs
#     n = np.array([-c0, -c1, 1])
#     n /= np.linalg.norm(n)

#     d = np.dot(pts3d - np.array([0, 0, c2]), n)[:, np.newaxis]
#     proj_pts = pts3d - d * n
#     return proj_pts


# def get_floor_contours(img):
#     H, W = img.shape[:2]
#     # Dummy trapezoid
#     return np.array([[(W//5, H), (2*W//5, H//2), (3*W//5, H//2), (4*W//5, H)]])
    
def main(vocab_path, settings_path, sequence_path):
    slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.MONOCULAR)
    slam.set_use_viewer(True)
    slam.initialize()
    
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    if not video_capture.isOpened():
        print("Error: Unable to open camera")
        return
    
    start_time = time.time()
    times_track = []
    Rts = None
    driveable_area = []
    while time.time() < (start_time+10000):
        ret_val, frame = video_capture.read()
        if not ret_val:
            continue

        t1 = time.time()
        slam.process_image_mono(frame, t1)
    
        mps = np.array(slam.get_tracked_mappoints())
        kps = np.array(slam.get_tracked_keypoints())

        if mps.size > 0 and kps.size > 0:
            # floor_inds = get_floor_points_kps(kps)
            floor_inds = get_floor_points_mps(mps)
            slam.set_floor_mappoints(floor_inds.tolist())

            K = slam.get_intrinsics_matrix()
            Rt = slam.get_extrinsics_matrix()
            Rts = np.concatenate([Rts, Rt[np.newaxis]], axis=0) if Rts is not None else Rt[np.newaxis]
            
            # Fit ground plane
            floor_mps = np.array(slam.get_floor_mappoints())
            plane_coefs = fit_plane_model(floor_mps)
            slam.set_ground_plane_params(*plane_coefs)

            # Project floor contours
            floor_contours = get_floor_contours(frame)
            for contour in floor_contours:
                pts3d = uv_to_plane3d(contour, K, Rt, plane_coefs)
                # print("***********************************")
                # for pt, mp in zip(pts3d, mps[floor_inds]):
                #     print(pt, mp)
                # print("***********************************")
                # self.slam.add_floor_contour(pts3d)
                pts2d = plane3d_to_2d(pts3d, plane_coefs)
                driveable_area.append(pts2d)
            np.save('driveable_area.npy', np.stack(driveable_area))

            # Project camera path
            camera_path = project_on_plane(Rts[:,:,3], plane_coefs)
            camera_path = plane3d_to_2d(camera_path, plane_coefs)
            np.save('camera_path.npy', camera_path)

        t2 = time.time()
        ttrack = t2 - t1
        times_track.append(ttrack)
        

    save_trajectory(slam.get_trajectory_points(), 'trajectory.txt')

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
    main(sys.argv[1], sys.argv[2], sys.argv[3])
