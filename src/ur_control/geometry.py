"""Small pure-python rotation helpers (UR axis-angle rotation vectors)."""
import math


def _norm(v):
    return math.sqrt(sum(c * c for c in v))


def rotvec_to_matrix(rv):
    th = _norm(rv)
    if th < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = (c / th for c in rv)
    c, s = math.cos(th), math.sin(th)
    t = 1 - c
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def matrix_to_rotvec(R):
    cos_th = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2))
    th = math.acos(cos_th)
    if th < 1e-9:
        return [0.0, 0.0, 0.0]
    if math.pi - th < 1e-6:  # ~180 deg: axis from diagonal
        n = [math.sqrt(max(0.0, (R[i][i] + 1) / 2)) for i in range(3)]
        k = max(range(3), key=lambda i: n[i])
        for j in range(3):
            if j != k and R[k][j] + R[j][k] < 0:
                n[j] = -n[j]
        return [c * math.pi for c in n]
    f = th / (2 * math.sin(th))
    return [f * (R[2][1] - R[1][2]), f * (R[0][2] - R[2][0]), f * (R[1][0] - R[0][1])]


def tool_z_axis(rv):
    """Tool Z axis (pointing out of the flange) expressed in the base frame."""
    R = rotvec_to_matrix(rv)
    return [R[0][2], R[1][2], R[2][2]]


def tool_down_rotvec(rv):
    """Rotation vector with tool Z pointing straight down (-Z base), keeping current heading."""
    R = rotvec_to_matrix(rv)
    hx, hy = R[0][0], R[1][0]            # tool X projected onto horizontal plane
    n = math.hypot(hx, hy)
    if n < 1e-6:
        hx, hy, n = 1.0, 0.0, 1.0
    x = (hx / n, hy / n, 0.0)
    z = (0.0, 0.0, -1.0)
    y = (x[1], -x[0], 0.0)               # z cross x
    return matrix_to_rotvec([[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]])


def angle_between(a, b):
    d = sum(p * q for p, q in zip(a, b)) / (_norm(a) * _norm(b))
    return math.acos(max(-1.0, min(1.0, d)))


def tool_to_base(rv, v):
    """Express a vector given in the tool frame in the base frame."""
    R = rotvec_to_matrix(rv)
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]
