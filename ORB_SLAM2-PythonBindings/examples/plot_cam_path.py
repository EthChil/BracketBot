import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv

if __name__ == '__main__':
    pts = np.load('camera_path.npy')
    contours = np.load('driveable_area.npy')
    # scatter plot of points going from blue to red chronologically
    for contour in contours:
        if cv.contourArea(contour[:,np.newaxis,:].astype(np.int32)) > 10:
            continue
        # Transparent yellow contour of the driveable area
        plt.fill(contour[:,0], contour[:,1], color='blue', alpha=0.1)
    plt.scatter(pts[:,0], pts[:,1], c=np.arange(len(pts)), cmap='viridis', zorder=10)
    plt.axis('equal')
    plt.show()