"""
This file contains a Python implementation of the narrow phase, which I will
use as pseudocode for the GPU kernel

Source: https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tritri_tam.pdf
"""
from collections import namedtuple
import numpy as np

def teq(self, other):
    return all(np.isclose(self.v0, other.v0)) and all(np.isclose(self.v1, other.v1)) and all(np.isclose(self.v2, other.v2))

Triangle = namedtuple("Triangle", ("v0", "v1", "v2"))
Triangle.__eq__ = teq

def compute_plane(triangle):
    v1_v0 = triangle.v1 - triangle.v0
    v2_v0 = triangle.v2 - triangle.v0
    N = np.cross(v1_v0, v2_v0)
    d = (-1 * N) @ triangle.v0

    return N, d

def compute_signed_dists(N, d, triangle):
    return np.array([N @ v + d for v in tuple(triangle)])

def no_overlap(dvs):
    return not (any(dvs >= 0) and any(dvs <= 0))

# O computed with https://math.stackexchange.com/questions/475953/how-to-calculate-the-intersection-of-two-planes
def compute_intersect_line(N1, d1, N2, d2):
    D = np.cross(N1, N2)

    # Set t = 1
    if D[2] != 0:
        A = np.array([N1[:2], N2[:2]])
        x = np.linalg.solve(A, np.array([-d1, -d2]))
        O = np.array([*x, 0])

    elif D[1] != 0:
        A = np.array([[N1[0], N1[2]], [N2[0], N2[2]]])
        x = np.linalg.solve(A, np.array([-d1, -d2]))
        O = np.array([x[0], 0, x[1]])

    else:
        A = np.array([N1[1:], N2[1:]])
        x = np.linalg.solve(A, np.array([-d1, -d2]))
        O = np.array([0, *x])

    return D, O

def project_vertex(V, D, O):
    return D @ (V - O)

def canonicalize_triangle(t, dvs):
    if np.sign(dvs[0]) == np.sign(dvs[1]):
        return Triangle(t.v0, t.v2, t.v1), np.array([dvs[0], dvs[2], dvs[1]])

    elif np.sign(dvs[0]) == np.sign(dvs[2]):
        return Triangle(t.v0, t.v1, t.v2), np.array([dvs[0], dvs[1], dvs[2]])

    else:
        return Triangle(t.v1, t.v0, t.v2), np.array([dvs[1], dvs[0], dvs[2]])

def compute_parametric_variable(v0, v1, d0, d1, D, O):
    p_v0 = project_vertex(v0, D, O)
    p_v1 = project_vertex(v1, D, O)

    return p_v0 + (p_v1 - p_v0) * d0 / (d0 - d1)

def is_coplanar(N1, d1, N2, d2):
    c1 = [*N1, d1]
    c2 = [*N2, d2]

    ratio = None
    for i in range(4):
        # If there is a situation where only one is zero, stop
        if np.isclose(c1[i], 0) ^ np.isclose(c2[i], 0):
            return False

        # If they are both close, skip
        elif np.isclose(c1[i], 0) and np.isclose(c2[i], 0):
            continue

        # Otherwise compare their ratios
        elif ratio is None:
            ratio = c1[i] / c2[i]

        elif not np.isclose(ratio, c1[i] / c2[i]):
            return False

    return True

def d_mnop(m, n, o, p):
    return np.sum((m - n) * (o - p))

# Source: http://paulbourke.net/geometry/pointlineplane/
def is_intersecting(p1, p2, p3, p4):
    # Checks if p1-p2 is intersected with p3-p4
    mua_denom = (d_mnop(p2, p1, p2, p1) * d_mnop(p4, p3, p4, p3) - d_mnop(p4, p3, p2, p1) * d_mnop(p4, p3, p2, p1))
    if mua_denom == 0:
        return False

    mua = (d_mnop(p1, p3, p4, p3) * d_mnop(p4, p3, p2, p1) - d_mnop(p1, p3, p2, p1) * d_mnop(p4, p3, p4, p3)) / mua_denom

    if (mua > 1 or mua < 0):
        return False

    mub_denom = d_mnop(p4, p3, p4, p3)
    if mub_denom == 0:
        return False

    mub = (d_mnop(p1, p3, p4, p3) + mua * d_mnop(p4, p3, p2, p1)) / mub_denom

    if (mub > 1 or mub < 0):
        return False

    Pa = p1 + mua * (p2 - p1)
    Pb = p3 + mub * (p4 - p3)

    return all(np.isclose(Pa, Pb))

def has_coplanar_intersection(t1, t2):
    for i in range(3):
        for j in range(i + 1, 3):
            for k in range(3):
                for l in range(k + 1, 3):
                    if is_intersecting(tuple(t1)[i], tuple(t1)[j], tuple(t2)[k], tuple(t2)[l]):
                        print(tuple(t1)[i], tuple(t1)[j], tuple(t2)[k], tuple(t2)[l])
                        return True

    return False


def is_contained(t1, t2):
    # Check if t1 is fully inside t2 or vice versa
    # Note: there is technically no reference for this, so I may have done
    # the geometry wrong

    t1_min_x = min(v[0] for v in tuple(t1))
    t1_min_y = min(v[1] for v in tuple(t1))
    t1_min_z = min(v[2] for v in tuple(t1))
    t1_max_x = max(v[0] for v in tuple(t1))
    t1_max_y = max(v[1] for v in tuple(t1))
    t1_max_z = max(v[2] for v in tuple(t1))

    t2_min_x = min(v[0] for v in tuple(t2))
    t2_min_y = min(v[1] for v in tuple(t2))
    t2_min_z = min(v[2] for v in tuple(t2))
    t2_max_x = max(v[0] for v in tuple(t2))
    t2_max_y = max(v[1] for v in tuple(t2))
    t2_max_z = max(v[2] for v in tuple(t2))

    if t1_min_x <= t2_min_x and t1_min_y <= t2_min_y and t1_min_z <= t2_min_z and \
            t1_max_x >= t2_max_x and t1_max_y >= t2_max_y and t1_max_z >= t2_max_z:
        return True

    if t1_min_x >= t2_min_x and t1_min_y >= t2_min_y and t1_min_z >= t2_min_z and \
            t1_max_x <= t2_max_x and t1_max_y <= t2_max_y and t1_max_z <= t2_max_z:
        return True

    return False


def is_overlapped(t1, t2):
    N2, d2 = compute_plane(t2)
    dvs1 = compute_signed_dists(N2, d2, t1)

    # Early exit
    if no_overlap(dvs1):
        return False

    N1, d1 = compute_plane(t1)

    # Handle coplanar interation separately
    if is_coplanar(N1, d1, N2, d2):
        return has_coplanar_intersection(t1, t2) or is_contained(t1, t2)

    # The two triangles are on different planes
    dvs2 = compute_signed_dists(N1, d1, t2)

    if no_overlap(dvs2):
        return False

    D, O = compute_intersect_line(N1, d1, N2, d2)

    ct1, cdvs1 = canonicalize_triangle(t1, dvs1)
    ct2, cdvs2 = canonicalize_triangle(t2, dvs2)

    t1_01 = compute_parametric_variable(ct1.v0, ct1.v1, cdvs1[0], cdvs1[1], D, O)
    t1_12 = compute_parametric_variable(ct1.v1, ct1.v2, cdvs1[1], cdvs1[2], D, O)

    t2_01 = compute_parametric_variable(ct2.v0, ct2.v1, cdvs2[0], cdvs2[1], D, O)
    t2_12 = compute_parametric_variable(ct2.v1, ct2.v2, cdvs2[1], cdvs2[2], D, O)

    if min(t1_01, t1_12) > max(t2_01, t2_12):
        return False
    elif min(t2_01, t2_12) > max(t1_01, t1_12):
        return False
    else:
        return True
