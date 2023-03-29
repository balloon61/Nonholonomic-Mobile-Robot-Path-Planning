#!/usr/bin/env python

from __future__ import annotations
import rospy

import numpy as np
from scipy.spatial.transform import Rotation
from typing import Union, List

class tf:

    def __init__(self, dim:int = 3) -> None:
        
        self.t_vec = np.zeros(3, )  # t_vec = [x, y, z]
        self.r_vec = np.zeros(3, )  # r_vec = [roll, pitch ,yaw]
        self.dim = dim
        
    def set_matrix(self, matrix: np.ndarray) -> tf:
        """
        Copies the matrix values into internal structure.
        """
        
        rot = Rotation.from_matrix(matrix[:3,:3]).as_rotvec()
        np.copyto(self.r_vec, rot)
        np.copyto(self.t_vec, matrix[:3, 3].flatten())

        return self

    def set_rotation(self, rotvec: Union[np.ndarray, List[float]]) -> tf:
        """
        Copies the rotation vector value(s) into internal structure.
        """
        rotvec = np.array(rotvec).flatten()
        np.copyto(self.r_vec, rotvec)

        return self

    # end def

    def set_translation(self, t_vec: Union[np.ndarray, List[float]]) -> tf:
        """
        Copies the translation vectors into internal structure.
        """
        t_vec = np.array(t_vec).flatten()
        np.copyto(self.t_vec, t_vec)

        return self

    # end def

    def transformation(self, goal:tf) -> tf:
        """
        
        """
        print(self.inverse_matrix().get_matrix())
        print(goal.get_matrix())
        T = self.inverse_matrix().get_matrix() @ goal.get_matrix()
        print(T)
        return tf(3).set_matrix(T)

    def inverse_matrix(self) -> tf:
        """
        Computes the inverse of the transformation matrix. Returns a new 
        transform.
        """
        mt = self.get_matrix()
        mt_inv = np.linalg.inv(mt)

        return tf(3).set_matrix(mt_inv)

    # end def

    def get_matrix(self) -> np.ndarray:
        """
        Returns a homogenous matrix of the transform.
        """
        # dim = len(self.t_vec)
        mt = np.eye(self.dim + 1)

        # Compute the rotation part.
        mt[:self.dim, :self.dim] = Rotation.from_rotvec(self.r_vec).as_matrix()[:self.dim, :self.dim]
        # Compute the translation part.
        mt[:self.dim, self.dim] = self.t_vec.flatten()

        return mt

    # end def

    def ndim(self) -> int:
        """
        Returns the number of dimensions in transform.
        """
        return len(self.t_vec)

    # end def

    def rotvec(self) -> np.ndarray:
        """
        Returns the rotation vector of the transform.
        """
        return self.r_vec.copy()

    # end def

    def tvec(self) -> np.ndarray:
        """
        Returns the translation vector of the transform.
        """
        return self.t_vec.copy()
    # end def
# end class
