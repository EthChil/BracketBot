import numpy as np
import scipy as sp
from skimage.measure.fit import BaseModel

"""
*** TAKEN FROM HERE ***
https://fpdpy.gitlab.io/fpd/_modules/fpd/ransac_tools.html
***********************
"""

class Plane3dModel(BaseModel):
    """Total least squares estimator for plane fit to 3-D point cloud.

    Attributes
    ----------
    model_data : array
        Fitted model.
    params : array
        model = C[0]*x + C[1]*y + C[2]

    """
    def predict(self, p, *args):
        m = p[0]*args[0] + p[1]*args[1] + p[2]
        return m

    def estimate(self, data):
        """Estimate model from data.

        Parameters
        ----------
        data : (N, D) array
            Flattened length N (x,y,z) point cloud to fit to.

        Returns
        -------
        success : bool
            True, if model estimation succeeds.
        """

        # best-fit linear plane
        # https://docs.scipy.org/doc/numpy-1.13.0/reference/generated/numpy.linalg.lstsq.html
        x, y, z = data.T
        A = np.c_[x, y, np.ones(data.shape[0])]
        C,_,_,_ = sp.linalg.lstsq(A, z)    # coefficients

        self.params = C
        return True

    def residuals(self, data):
        """Determine residuals of data to model.

        Parameters
        ----------
        data : (N, D) array
            Flattened length N (x,y,z) point cloud to fit to.

        Returns
        -------
        residuals : (N) array
            Z residual for each data point.

        """

        x, y, z = data.T
        C = self.params
        # evaluate model at points
        args = (x, y, z)
        self.model_data = self.predict(self.params, *args)
        z_error = self.model_data - z
        return z_error