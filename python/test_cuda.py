from narrow import *

def main():
    pts = [np.array([1, 2, 3]),
           np.array([2, 2, 3.5]),
           np.array([2, 2.5, 3]),
           np.array([1, 3, 4]),
           np.array([4, 7, 2]),
           np.array([5, 1, 3])]
    t1 = Triangle(pts[0], pts[1], pts[2])
    N1, d1 = compute_plane(t1);
    print("compute_plane corr:", compute_plane(t1))

    t2 = Triangle(pts[3], pts[4], pts[5])
    dists12 = compute_signed_dists(N1, d1, t2)
    print("compute_signed_dists corr:", dists12)

    print("no_overlap(t1, t2):", no_overlap(dists12));
    print("yes_overlap(t1, t2):", yes_overlap(dists12));

    N2, d2 = compute_plane(t2)
    D, O = compute_intersect_line(N1, d1, N2, d2)
    dists21 = compute_signed_dists(N2, d2, t1)

    ct1, cdists21 = canonicalize_triangle(t1, dists21)
    print("compute_parametric_variable(ct1.v0, ct1.v1, cdists21[0], cdists21[1], D, O):", compute_parametric_variable(ct1.v0, ct1.v1, cdists21[0], cdists21[1], D, O))
    print("compute_parametric_variable(ct1.v1, ct1.v2, cdists21[1], cdists21[2], D, O):", compute_parametric_variable(ct1.v1, ct1.v2, cdists21[1], cdists21[2], D, O))
    ct2, cdists12 = canonicalize_triangle(t2, dists12)
    print("compute_parametric_variable(ct2.v0, ct2.v1, cdists12[0], cdists12[1], D, O):", compute_parametric_variable(ct2.v0, ct2.v1, cdists12[0], cdists12[1], D, O))
    print("compute_parametric_variable(ct2.v1, ct2.v2, cdists12[1], cdists12[2], D, O):", compute_parametric_variable(ct2.v1, ct2.v2, cdists12[1], cdists12[2], D, O))

if __name__ == "__main__":
    main()
