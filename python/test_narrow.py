import numpy

from narrow import *

def test_all():
    # Checked against https://math.stackexchange.com/questions/562123/equation-of-plane-containing-two-vectors
    v0 = np.array([1, 2, 3])
    u = np.array([1, 0, np.sqrt(3)])
    v = np.array([1, np.sqrt(3), 0])
    t1 = Triangle(v0, v0 + u, v0 + v)

    N1, d1 = compute_plane(t1)
    assert all(np.isclose(N1, np.array([-3, np.sqrt(3), np.sqrt(3)])))
    assert np.isclose(d1, 3 + -2 * np.sqrt(3) + -3 * np.sqrt(3))

    w = np.array([0, 1, np.sqrt(3)])
    t2 = Triangle(v0, v0 + u, v0 + w)
    dvs2 = compute_signed_dists(N1, d1, t2)
    # I realize this test is a little sketchy...
    assert all(np.isclose(dvs2, np.array([0, 0, N1 @ (v0 + w) + d1])))

    # There is definitely overlap (sharing a vertex)
    assert not no_overlap(dvs2)
    assert yes_overlap(dvs2)

    # Maybe have overlap
    t3 = Triangle(np.array([1, 1, 1]), np.array([0.5, 2, 3]), np.array([3, 0, 0]))
    dvs3 = compute_signed_dists(N1, d1, t3)
    assert not no_overlap(dvs3)
    assert not yes_overlap(dvs3)

    # Definitely do not have overlap
    t4 = Triangle(np.array([300, 1, 1]), np.array([300, 2, 3]), np.array([300, 0, 0]))
    dvs4 = compute_signed_dists(N1, d1, t4)
    assert no_overlap(dvs4)
    assert not yes_overlap(dvs4)

    # Canonicalize the triangle
    ct3, cdvs3 = canonicalize_triangle(t3, dvs3)
    assert ct3 == t3
    assert all(np.isclose(cdvs3, dvs3))

    N3, d3 = compute_plane(t3)
    D, O = compute_intersect_line(N1, d1, N3, d3)
    dvs1 = compute_signed_dists(N3, d3, t1)

    # Note: I realize that the testing of these numbers is a bit sketchy
    ct1, cdvs1 = canonicalize_triangle(t1, dvs1)
    t1_01 = compute_parametric_variable(ct1.v0, ct1.v1, cdvs1[0], cdvs1[1], D, O)
    assert np.isclose(t1_01, -67.26042329704889)
    t1_12 = compute_parametric_variable(ct1.v1, ct1.v2, cdvs1[1], cdvs1[2], D, O)
    assert np.isclose(t1_12, -85.69040061472924)
    t3_01 = compute_parametric_variable(ct3.v0, ct3.v1, cdvs3[0], cdvs3[1], D, O)
    assert np.isclose(t3_01, -48.46170455690249)
    t3_12 = compute_parametric_variable(ct3.v1, ct3.v2, cdvs3[1], cdvs3[2], D, O)
    assert np.isclose(t3_12, -51.68157339273106)

    # This means that there is no overlap
    assert not is_overlapped(t1, t3)

    # Two triangles that we know are overlapped
    t5 = Triangle(np.array([0, 0, 0]), np.array([0, 0, 100]), np.array([0, 100, 0]))
    t6 = Triangle(np.array([1, 1, 1]), np.array([-1, 1, 1]), np.array([-1, 1, -1]))
    assert is_overlapped(t5, t6)

    # Early exit, no overlap
    assert not is_overlapped(t1, t4)

    # Early exit, yes overlap
    assert is_overlapped(t1, t2)

def test_compute_intersect_line():
    XY_plane = np.array([0, 0, 1])
    YZ_plane = np.array([1, 0, 0])
    XZ_plane = np.array([0, 1, 0])

    D, O = compute_intersect_line(XY_plane, -1, YZ_plane, -2)
    assert all(np.isclose(D, np.array([0, 1, 0])))
    assert all(np.isclose(O, np.array([2, 0, 1])))

    D, O = compute_intersect_line(YZ_plane, -1, XZ_plane, -2)
    assert all(np.isclose(D, np.array([0, 0, 1])))
    assert all(np.isclose(O, np.array([1, 2, 0])))

    D, O = compute_intersect_line(XZ_plane, -1, XY_plane, -2)
    assert all(np.isclose(D, np.array([1, 0, 0])))
    assert all(np.isclose(O, np.array([0, 1, 2])))

    # Example from https://math.stackexchange.com/questions/475953/how-to-calculate-the-intersection-of-two-planes
    D, O = compute_intersect_line(np.array([2, 3, -2]), 2, np.array([1, 2, 1]), -1)
    assert all(np.isclose(D, np.array([7, -4, 1])))
    assert all(np.isclose(O, np.array([-7, 4, 0])))

def test_project_vertex():
    V = np.array([1, 2, 2])
    O = np.array([2, 2, 4])
    D = np.array([3, 4, 5])

    p = project_vertex(V, D, O)
    assert np.isclose(p, -13)

def test_canonicalize_triangle():
    v0 = np.array([3, 1, 1])
    v1 = np.array([1, 2, 3])
    v2 = np.array([2, 0, 0])

    t, dvs = canonicalize_triangle(Triangle(v0, v1, v2), [-1, -1, 1])
    assert t == Triangle(v0, v2, v1)
    assert all(np.isclose(dvs, np.array([-1, 1, -1])))

    t, dvs = canonicalize_triangle(Triangle(v0, v1, v2), [-1, 1, 1])
    assert t == Triangle(v1, v0, v2)
    assert all(np.isclose(dvs, np.array([1, -1, 1])))

    t, dvs = canonicalize_triangle(Triangle(v0, v1, v2), [-1, 1, -1])
    assert t == Triangle(v0, v1, v2)
    assert all(np.isclose(dvs, np.array([-1, 1, -1])))
