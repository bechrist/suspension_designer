"""Geometry Utility Functions"""
from __future__ import annotations

import typing as typ
import numpy.typing as npt

import operator as op

import numpy as np

import scipy.linalg as sla
import scipy.spatial.transform as sptl

from suspension_designer.utilities import ordered_unique, sequence_to_index

__all__ = ['lerp',                                  # interpolation
           'Line', 'Plane',                         # linear subspaces
           'EulerRotation',                         # rotations
           'vector_alignment_angles',               # "
           'vector_alignment_rotation',             # "
           'skew_symmetric_matrix']                 # "

# %% Interpolation
def lerp(point_A: np.ndarray, point_B: np.ndarray, alpha: float) -> np.ndarray:
    """Linear interpolation between two points

    :param point_A: First point
    :type point_A: numpy.ndarray
    
    :param point_B: Second point
    :type point_B: numpy.ndarray
    
    :param alpha: Interpolation scalar
    :type alpha: float
    
    :return: Linear interpolation result
    :rtype: numpy.ndarray
    """
    return point_A*(1-alpha) + point_B*alpha

# %% Linear Subspaces
class Line():
    """A 1D line in :math:`\mathbb{R}^{dimension}`
    
    :param point_A: First point defining line
    :type point_A: numpy.ndarray
    
    :param point_B: Second point defining line
    :type point_B: numpy.ndarray
    """
    def __init__(self, point_A: np.ndarray, point_B: np.ndarray):
        """Initialize Line"""
        self.point = np.array([point_A, point_B])
        self.basis = (point_B - point_A) / np.linalg.norm(point_B - point_A)
        self.dimension = self.point.shape[-1]

    def __call__(self, query: float, index: int = 0) -> np.ndarray:
        """Evaluates line between two points given coordinate and index

        :param query: Query coordinate
        :type query: float
        
        :param index: Query coordinate index, defaults to 0
        :type index: int, optional
        
        :return: Linear interpolation result
        :rtype: numpy.ndarray
        """
        alpha = (query - self.point[0][index]) \
            / (self.point[1][index] - self.point[0][index])
        return lerp(self.point[0], self.point[1], alpha)

    def proj(self, point: np.ndarray) -> np.ndarray:
        """Orthogonal projection onto the line
        
        :param point: Point of interest
        :type point: numpy.ndarray
        
        :return: Point projection onto line
        :rtype: numpy.ndarray
        """
        return self.point[0] + np.dot(point - self.point[0], self.basis) * self.basis
    
    def perp(self, point: np.ndarray) -> np.ndarray:
        """Perpendicular component to the orthogonal projection onto the line
        
        :param point: Point of interest
        :type point: numpy.ndarray
        
        :return: Perpendicular component to the projection of point from the line
        :rtype: numpy.ndarray
        """
        return point - self.proj(point)

class Plane():
    """A 2D plane in :math:`\mathbb{R}^{3}`
    
    :param point_A: First point defining plane
    :type point_A: numpy.ndarray
    
    :param point_B: Second point defining plane
    :type point_B: numpy.ndarray

    :param point_C: Third point defining plane
    :type point_C: numpy.ndarray
    """
    def __init__(self, point_A: np.ndarray, point_B: np.ndarray, point_C: np.ndarray):
        """Initialize Plane"""
        self.point = np.array([point_A, point_B, point_C])
        self.dimension = self.point.shape[-1]
        if self.dimension != 3:
            raise ValueError('Provided points are not three dimensional')
        
        A = np.hstack((self.point, np.ones((3,1))))
        self.null_space = sla.null_space(A).squeeze()
        if len(self.null_space.shape) != 1:
            raise ValueError('Provided points do not create a valid plane')
        
        self.n = self.null_space[:3] / np.linalg.norm(self.null_space[:3])
        
    def __call__(self, query: typ.Collection[float, float], 
                 index: typ.Collection[int, int] = (0,1)) -> np.ndarray:
        """Evaluate plane at query coordinates for given dimension indices
        
        :param query: Query coordinates
        :type query: typing.Collection[float, float]

        :param index: Query coordinate indices, defaults to (0,1)
        :type index: typing.Collection[int, int], optional

        :return: Planar evaluation
        :rtype: numpy.ndarray
        """
        if index != ordered_unique(index) or len(index) != 2:
            raise ValueError('Invalid index set')

        point = np.empty((3,))
        for j, qj in zip(index, query):
            point[j] = qj
        
        i = [i for i in range(3) if i not in index][0]
        point[i] = (-self.null_space[-1] - np.dot(self.null_space[[index]], query)) \
            / self.null_space[i] 

        return point

    def intersection(self, other: Plane) -> Line:
        """Compute linear intersection of two planes
        
        :param other: Intersecting plane
        :type other: Plane
        
        :return: Intersection line
        :rtype: Line
        """
        A = np.array([self.null_space, other.null_space])
        j = np.unravel_index(np.argmin(np.abs(A[:,:3])), A[:,:3].shape)[1]

        Ar = A[:,[jj for jj in range(3) if jj != j]]
        b0 = A[:,-1]
        b1 = A[:,-1] + A[:,j]

        point_A = np.insert(np.linalg.solve(Ar, -b0), 0, 0)
        point_B = np.insert(np.linalg.solve(Ar, -b1), 0, 1)

        return Line(point_A, point_B)
    
    def proj(self, point: np.ndarray) -> np.ndarray:
        """Orthogonal projection onto the plane
        
        :param point: Point of interest
        :type point: numpy.ndarray
        
        :return: Point projection onto plane
        :rtype: numpy.ndarray
        """
        return point - self.perp(point)

    def perp(self, point: np.ndarray) -> np.ndarray:
        """Perpendicular component to the orthogonal projection onto the plane
        
        :param point: Point of interest
        :type point: numpy.ndarray
        
        :return: Perpendicular component to orthogonal projection onto plane
        :rtype: numpy.ndarray
        """
        return self.point[0] + np.dot(point - self.point[0], self.n) * self.n

# %% Rotations
class EulerRotation(np.ndarray):
    """Euler / Tait-Bryan angle based rotations. Subclasses :code:`numpy.ndarray` 
    and wraps a :code:`scipy.spatial.transform.Rotation` object for efficient
    rotation computations.

    :param rotation: Euler / Tait-Bryan angles
    :type rotation: numpy.typing.Arraylike

    :param sequence: Rotation angle sequence, defaults to 'ZYX' (intrinsic)
    :type sequence: str, optional

    :param degrees: Unit of rotation angles, defaults to True
    :type degrees: bool, optional
    """
    def __new__(cls, rotation: npt.ArrayLike, 
                sequence: str = 'ZYX', degrees: bool = True) -> EulerRotation:
        """See: https://numpy.org/doc/stable/user/basics.subclassing.html"""
        obj = np.asarray(rotation).view(cls)

        obj.sequence = sequence
        obj.degrees = degrees

        obj._operator_synced = False
        obj._operator = None

        return obj
    
    def __array_finalize__(self, obj: np.ndarray | None):
        """See: https://numpy.org/doc/stable/user/basics.subclassing.html"""
        if obj is None: return
        
        self._operator = None

        self._sequence = getattr(obj, '_sequence', 'ZYX')
        self.sequence = getattr(obj, 'sequence', self._sequence)

        self._index = getattr(obj, '_index', sequence_to_index(self.sequence))

        self.degrees = getattr(obj, 'degrees', True)
        
        # running self._set_operator() here incurs a recursion loop?
        self._operator_synced = getattr(obj, '_operator_synced', False)
        self._operator = getattr(obj, '_operator', None)
    
    def __setitem__(self, key: int | slice, value: float):
        """Invoke :code:`numpy.ndarray` `__setitem__()` dunder and then set 
        corresponding operator."""
        super().__setitem__(key, value)

        self._operator_synced = False

    sequence: str = property(op.attrgetter('_sequence'))
    
    @sequence.setter
    def sequence(self, value: str):
        """Sets sequence field and computes correct corresponding indexing list

        :param value: Sequence value
        :type value: str
        """
        self._index = sequence_to_index(value)
        self._sequence = value

        self._operator_synced = False
    
    def __str__(self) -> str:
        return (
            f"EulerRotation(" 
            f"[{self[0]}, {self[1]}, {self[2]}], {self.sequence}, " 
            f"degrees={self.degrees}, synced={self._operator_synced})")
    
    def __repr__(self) -> str:
        return str(self)

    def _set_operator(self):
        """Set :code:`scipy.spatial.transform.Rotation` operator attribute from 
        rotation angles, sequence, and degree flag."""
        self._operator = sptl.Rotation.from_euler(
            self.sequence, self[self._index], self.degrees)
        
        self._operator_synced = True

    def _sync_operator(function: typ.Callable) -> typ.Any:
        """Decorator to set operator if it is not synced
        
        :param function: Callable to be decorated
        :type function: typing.Callable
        
        :return: Callable's return
        :rtype: typing.Any
        """
        def decorator(*args, **kwargs):
            self: EulerRotation = args[0]
            if not self._operator_synced:
                self._set_operator()
            return function(*args, **kwargs)
        return decorator

    @_sync_operator
    def apply(self, vector: npt.ArrayLike, inverse: bool = False) -> np.ndarray:
        """Applies forward rotation to input vector(s)
        
        :param vector: Input vector(s) of shape :math:`(n,3)` where :math:`n` is
            the number of vectors to be rotated 
        :type vector: numpy.ndarray
        
        :param inverse: Inverse rotation flag, defaults to :code:`False`
        :type inverse: bool, optional
        
        :return: Rotated vectors with shape :math:`(n,3)`
        :rtype: numpy.ndarray
        """        
        return self._operator.apply(vector, inverse=inverse)

    @_sync_operator
    def as_matrix(self) -> np.ndarray:
        """Returns rotation matrix corresponding to rotation
        
        :return: Rotation matrix
        :rtype: numpy.ndarray
        """
        return self._operator.as_matrix()
    
def vector_alignment_angles(v1: np.ndarray, v2: np.ndarray,
                            sequence: str = 'ZX') -> tuple[float,float]:
    """Provides two Euler angles that align a pair of provided 3-vectors.
    Adapted from https://stackoverflow.com/a/67455855.

    :param v1: Initial vector
    :type: v1: numpy.ndarray

    :param v2: Target vector
    :type: v2: numpy.ndarray

    :param sequence: Rotation sequence
    :type sequence: str

    :return: Pair of Euler angles for specified axes
    :rtype: tuple[float, float]
    """
    # Append null axis
    idx = sequence_to_index(sequence)
    idx.append(*[i for i in range(3) if i not in idx])

    r1 = np.linalg.norm(v1)
    r2 = np.linalg.norm(v2)

    z1, x1, y1 = v1[idx]             # initial vector
    z2, x2, y2 = v2[idx] * (r1/r2)   # scaled target vector

    # Elevation rotation
    rho1 = np.sqrt(z1**2 + y1**2)
    if(abs(z2 / rho1) > 1):
        raise ValueError('Vectors can not be aligned with two Euler angles')
    phi = np.arcsin(z2 / rho1) - np.arctan2(z1, y1);

    y1, z1 = (y1 * np.cos(phi) - z1 * np.sin(phi), 
              y1 * np.sin(phi) + z1 * np.cos(phi))
    np.allclose(rho1, np.sqrt(z1**2 + y1**2))

    # Azimuthal rotation
    theta = np.arctan2(y2, x2) - np.arctan2(y1, x1)
    x1, y1 = (x1 * np.cos(theta) - y1 * np.sin(theta), 
              x1 * np.sin(theta) + y1 * np.cos(theta))
    
    # Check alignment
    assert np.allclose([x1, y1, z1], [x2, y2, z2])
    
    return theta, phi

def vector_alignment_rotation(v_A: np.ndarray, v_B: np.ndarray) -> sptl.Rotation:
    """Provide a rotation matrix to align the first vector onto the second.
    Adapted from https://math.stackexchange.com/a/476311

    :param v_A: Misaligned vector
    :type v_A: numpy.ndarray

    :param v_B: Reference direction vector
    :type v_B: numpy.ndarray

    :return: Alignment rotation
    :rtype: scipy.spatial.transform.Rotation
    """
    v = np.cross(v_A, v_B)
    c = np.dot(v_A, v_B)
         
    v_skew = skew_symmetric_matrix(v)
    R = np.eye(3) + v_skew + (v_skew @ v_skew) / (1 + c)
    return sptl.Rotation.from_matrix(R)

# %% Helpers
def skew_symmetric_matrix(v: np.ndarray) -> np.ndarray:
    r"""Creates skew symmetric cross-product matrix corresponding 
    to vector in :math:`\mathbb{R}^3`

    :param v: Input vector
    :type v: numpy.ndarray

    :return: Skew symmetric cross-product matrix
    :rtype: numpy.ndarray
    """
    return np.array([[ 0   , -v[2],  v[1]],
                     [ v[2],  0   , -v[0]],
                     [-v[1],  v[0],  0   ]])
