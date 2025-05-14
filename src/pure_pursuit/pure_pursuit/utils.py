import numpy as np


def segment_circle_intersection(p1, v, p_o, r, min_t=0.0):
    """
    Find the first intersection between the circle centered at p_o with a radius equal to the r
    distance, and the line segment in the vector form p1+tv.

    Circle equation: |p-p_o|=r
    line segment equation: p = p1+t*(p2-p1) = t*v + p1, t in [0,1]
    -> (t*v+d)^2 = r^2, where d=p1-p_o
    -> solve v^2*t^2 + 2*d*v*t + d^2 - r^2 = 0, t in [0,1], optional constraint: t>=min_t

    Parameters
    ----------
    p1: ndarray
        (2,), starting point of the line segment.
    v: ndarray
        (2,), line segment vector from starting point to end point.
    p_o: ndarray
        (2,), circle center.
    r: float
        Circle radius.
    min_t: float
        If a minimum distance from the start of the line segment is required for the target intersection point.
    Returns
    -------
    ndarray or None
        An (2,) array denoting the first intersection point or None if no intersection found.
    """
    d = p1 - p_o
    a = np.dot(v, v)
    b = 2.0 * np.dot(d, v)
    c = np.dot(d, d) - r**2

    delta = b**2 - 4*a*c
    if delta < 0:  # no equation solution
        return None
    sqrt_delta = np.sqrt(delta)
    for t in sorted(((-b - sqrt_delta) / (2 * a), (-b + sqrt_delta) / (2 * a))):
        if 0.0 <= t <= 1.0 and t >= min_t: # the first solution satisfying the constraints
            return p1 + t * v
    return None
