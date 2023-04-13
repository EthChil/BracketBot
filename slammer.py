from sklearn.linear_model import LinearRegression
from sklearn.linear_model import RANSACRegressor
import numpy as np


def plot_floor_pts(floor_mps, plane_model):
    # Import outside breaks ORBSLAM
    import matplotlib as plt
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    n = 20
    x, y, z = floor_mps.T
    ax.scatter(x, y, z, s=0.01, c='r')

    xx = np.linspace(x.min(), x.max(), n)
    yy = np.linspace(y.min(), y.max(), n)
    X, Y = np.meshgrid(xx, yy)
    Z = c0*X.flatten() + c1*Y.flatten() + c2
    Z = Z.reshape(X.shape)
    ax.plot_surface(X, Y, Z, shade=False)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()


def get_floor_points_kps(kps):
    if len(kps) == 0:
        return []

    floor_inds = []
    for i, (kpx, kpy) in enumerate(kps):
        kpx = int(kpx)
        kpy = int(kpy)
        if 400 < kpx < 700 and kpy > 150:
            floor_inds.append(i)

    return floor_inds


def get_floor_points_mps(mps):
    if len(mps) == 0:
        return []

    zs = mps[:,1]
    z_top10 = np.percentile(zs, 90)
    floor_inds = np.argwhere(zs > z_top10)

    return floor_inds.squeeze()


def fit_plane_model(pts):
    lin_reg = LinearRegression(fit_intercept=True)
    ransac = RANSACRegressor(base_estimator=lin_reg,
                             min_samples=10,
                             residual_threshold=0.01,
                             max_trials=1000)
    xy = pts[:,:2]
    z = pts[:,2]
    ransac.fit(xy, z)
    
    c0, c1 = ransac.estimator_.coef_
    c2 =  ransac.estimator_.intercept_
    coefs = [float(c0), float(c1), float(c2)]
    return coefs


def uv_to_plane3d(pts2d, K, Rt, plane_coefs):
    """
    :param pts2d: points to project (u, v)
    :param K: Camera intrinsics matrix
    :param Rt: Camera extrinsics matrix
    :param plane_model: Plane3DModel object, output of fit_plane_model()
    :return: 3D points representing projection of pts2d onto the floor plane in the 3D map space
    """

    pts3d = []

    c0, c1, c2 = plane_coefs
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    R = Rt[:,:3]
    t = Rt[:,3]

    # Ax = b
    A = np.array([[-1, 0, 0],[0, -1, 0],[-c0, -c1, 0]])
    b = np.array([0, 0, c2]).T
    for (u, v) in pts2d:
        x_ = (u-cx)/fx
        y_ = (v-cy)/fy
        P = R.T @ np.array([x_, y_, 1]).T
        Q = R.T @ t

        A[:,2] = P
        assert np.linalg.det(A) != 0

        X, Y, z = np.linalg.inv(A) @ (Q+b)
        Z = c0*X + c1*Y + c2
        pt3d = np.array([X, Y, Z])
        # Check that z is correct
        # assert np.allclose(z*P - Q, pt3d, rtol=1e-6)

        # Check that the projection is correct
        screen_pt = K @ Rt @ np.append(pt3d, 1)
        uv_proj = screen_pt[:2]/screen_pt[2]
        # assert np.allclose(uv_proj, [u, v], rtol=1e-6)

        pts3d.append(pt3d)

    return np.stack(pts3d)


def plane3d_to_2d(pts3d, plane_coefs):
    """
    :param pts3d: points to project (x, y, z)
    :param plane_model: Plane3DModel object, output of fit_plane_model()
    :return: 2D points representing projection of pts3d onto the ground plane
    """
    c0, c1, c2 = plane_coefs
    origin = np.array([0, 0, c2])

    # Forward, right and normal direction vectors for the ground plane
    fwd = np.array([0, 1/c1, c2+1]) - origin
    fwd /= np.linalg.norm(fwd)
    right = np.array([1, (c2-c0)/c1, c2]) - origin
    right /= np.linalg.norm(right)

    d_right = np.dot(pts3d - origin, right)
    d_fwd = np.dot(pts3d - origin, fwd)
    pts2d = np.stack([d_right, d_fwd], axis=1)
    return pts2d


def project_on_plane(pts3d, plane_coefs):
    c0, c1, c2 = plane_coefs
    n = np.array([-c0, -c1, 1])
    n /= np.linalg.norm(n)

    d = np.dot(pts3d - np.array([0, 0, c2]), n)[:, np.newaxis]
    proj_pts = pts3d - d * n
    return proj_pts


def get_floor_contours(img):
    H, W = img.shape[:2]
    # Dummy trapezoid
    return np.array([[(W//5, H), (2*W//5, H//2), (3*W//5, H//2), (4*W//5, H)]])


if __name__ == '__main__':
    all_mps = np.loadtxt('all_mappoints.txt')
    floor_mps = np.loadtxt('floor_mappoints.txt')

    plane_coefs = fit_plane_model(floor_mps)
    plot_floor_pts(floor_mps, plane_coefs)
