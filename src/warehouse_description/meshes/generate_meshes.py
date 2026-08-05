#!/usr/bin/env python3
"""
Generate STL mesh files for the Warehouse AMR robot.

Run from this directory:  python3 generate_meshes.py
Output: amr_body.stl, amr_lid.stl, amr_bumper_front.stl, amr_bumper_rear.stl,
        amr_wheel.stl, amr_mast.stl, amr_sensor_ring.stl,
        amr_beacon.stl, amr_dome.stl
"""

import math, struct, os

OUT = os.path.dirname(os.path.abspath(__file__))


# ── Binary STL writer ─────────────────────────────────────────────────────────

def write_stl(name, tris):
    path = os.path.join(OUT, name)
    with open(path, 'wb') as f:
        hdr = b'Warehouse AMR mesh: ' + name.encode() + b'\x00' * 80
        f.write(hdr[:80])
        f.write(struct.pack('<I', len(tris)))
        for v0, v1, v2 in tris:
            e1 = [v1[k] - v0[k] for k in range(3)]
            e2 = [v2[k] - v0[k] for k in range(3)]
            n = [e1[1]*e2[2] - e1[2]*e2[1],
                 e1[2]*e2[0] - e1[0]*e2[2],
                 e1[0]*e2[1] - e1[1]*e2[0]]
            m = math.sqrt(sum(x*x for x in n)) or 1.0
            n = [x / m for x in n]
            f.write(struct.pack('<fff', *n))
            f.write(struct.pack('<fff', *v0))
            f.write(struct.pack('<fff', *v1))
            f.write(struct.pack('<fff', *v2))
            f.write(struct.pack('<H', 0))
    print(f"  {name:30s}  {len(tris):5d} triangles")


# ── Primitive builders ────────────────────────────────────────────────────────

def ring(r, n, z):
    """n-point ring of radius r at height z."""
    return [(r * math.cos(2*math.pi*k/n),
             r * math.sin(2*math.pi*k/n), z)
            for k in range(n)]


def cap(pts, flip=False):
    """Fan-triangulate a convex polygon cap."""
    tris = []
    for i in range(1, len(pts) - 1):
        t = (pts[0], pts[i + 1], pts[i])
        tris.append(t[::-1] if flip else t)
    return tris


def tube(top, bot):
    """Quad-strip between two equal-length rings."""
    tris, n = [], len(top)
    for i in range(n):
        j = (i + 1) % n
        tris += [(top[i], bot[i], bot[j]),
                 (top[i], bot[j], top[j])]
    return tris


def cylinder(r, h, segs=32, cz=0.0):
    t = ring(r, segs, cz + h / 2)
    b = ring(r, segs, cz - h / 2)
    return cap(t) + cap(b, flip=True) + tube(t, b)


def frustum(r_top, r_bot, h, segs=16, cz=0.0):
    t = ring(r_top, segs, cz + h / 2)
    b = ring(r_bot, segs, cz - h / 2)
    return cap(t) + cap(b, flip=True) + tube(t, b)


def chamfered_box(lx, ly, lz, c, cz=0.0):
    """
    Box with chamfered edges — octagonal cross-section, full 3-D bevel.
    lx/ly/lz = full extents, c = bevel size.
    """
    hx, hy, hz = lx / 2, ly / 2, lz / 2

    # 8-point XY cross-section (CCW, +x first)
    xy = [
        ( hx - c,  hy), ( hx,  hy - c),
        ( hx, -(hy - c)), ( hx - c, -hy),
        (-(hx - c), -hy), (-hx, -(hy - c)),
        (-hx,  hy - c), (-(hx - c),  hy),
    ]
    top = [(x, y, cz + hz) for x, y in xy]
    bot = [(x, y, cz - hz) for x, y in xy]
    return cap(top) + cap(bot, flip=True) + tube(top, bot)


def torus(R, r, seg_M=32, seg_m=16, cz=0.0):
    """Torus / sensor ring."""
    verts = []
    for i in range(seg_M):
        a_M = 2 * math.pi * i / seg_M
        for j in range(seg_m):
            a_m = 2 * math.pi * j / seg_m
            x = (R + r * math.cos(a_m)) * math.cos(a_M)
            y = (R + r * math.cos(a_m)) * math.sin(a_M)
            z = cz + r * math.sin(a_m)
            verts.append((x, y, z))
    tris = []
    for i in range(seg_M):
        for j in range(seg_m):
            ni = (i + 1) % seg_M
            nj = (j + 1) % seg_m
            a = verts[i * seg_m + j]
            b = verts[ni * seg_m + j]
            c = verts[i * seg_m + nj]
            d = verts[ni * seg_m + nj]
            tris += [(a, b, d), (a, d, c)]
    return tris


def sphere(r, segs=24, cz=0.0):
    """UV sphere."""
    rings_n = segs // 2
    rows = [ring(r * math.sin(math.pi * i / rings_n), segs,
                 cz + r * math.cos(math.pi * i / rings_n))
            for i in range(rings_n + 1)]
    tris = []
    for i in range(rings_n):
        top_r, bot_r = rows[i], rows[i + 1]
        n = len(top_r)
        if i == 0:
            for j in range(n):
                tris.append((top_r[0], bot_r[(j + 1) % n], bot_r[j]))
        elif i == rings_n - 1:
            for j in range(n):
                tris.append((top_r[j], top_r[(j + 1) % n], bot_r[0]))
        else:
            tris += tube(top_r, bot_r)
    return tris


def wheel_mesh(r, w, segs=32, tread_n=16):
    """
    Detailed wheel: smooth outer tyre with tread grooves,
    flat spoked side faces.
    """
    hw = w / 2
    tris = []
    spoke_r = r * 0.48

    # Tread: alternating raised/lowered bands
    r_hi, r_lo = r, r * 0.966
    n_bands = tread_n * 2
    prev_r = r_lo  # first strip at hi will have transition from lo
    for t in range(n_bands):
        cur_r = r_hi if t % 2 == 0 else r_lo
        nxt_r = r_lo if t % 2 == 0 else r_hi
        a1 = 2 * math.pi * t / n_bands
        a2 = 2 * math.pi * (t + 1) / n_bands
        c1, s1 = math.cos(a1), math.sin(a1)
        c2, s2 = math.cos(a2), math.sin(a2)
        # Radial transition wall at a1
        vtl = (prev_r * c1, prev_r * s1,  hw)
        vtc = (cur_r  * c1, cur_r  * s1,  hw)
        vbl = (prev_r * c1, prev_r * s1, -hw)
        vbc = (cur_r  * c1, cur_r  * s1, -hw)
        tris += [(vtl, vtc, vbc), (vtl, vbc, vbl)]
        # Outer band face
        vtm = (cur_r * c2, cur_r * s2,  hw)
        vbm = (cur_r * c2, cur_r * s2, -hw)
        tris += [(vtc, vbc, vbm), (vtc, vbm, vtm)]
        prev_r = cur_r

    # Flat side faces: hub disc + spokes implied by solid fill
    for i in range(segs):
        a1 = 2 * math.pi * i / segs
        a2 = 2 * math.pi * (i + 1) / segs
        # Left side (+hw): from center to outer rim
        o = (0.0, 0.0, hw)
        p1 = (r * math.cos(a1), r * math.sin(a1), hw)
        p2 = (r * math.cos(a2), r * math.sin(a2), hw)
        tris.append((o, p2, p1))
        # Right side (-hw)
        o = (0.0, 0.0, -hw)
        p1 = (r * math.cos(a1), r * math.sin(a1), -hw)
        p2 = (r * math.cos(a2), r * math.sin(a2), -hw)
        tris.append((o, p1, p2))

    return tris


# ── Generate every mesh ───────────────────────────────────────────────────────

print("Generating Warehouse AMR robot meshes...")
print()

# Main orange body (chamfered box, 0.9×0.65×0.4 m, body_z=0.045 in base_link frame)
write_stl("amr_body.stl",
          chamfered_box(0.900, 0.650, 0.400, c=0.040, cz=0.045))

# Top dark lid plate (slightly overhanging)
write_stl("amr_lid.stl",
          chamfered_box(0.920, 0.670, 0.026, c=0.018, cz=0.258))

# Front bumper bar (dark)
write_stl("amr_bumper_front.stl",
          chamfered_box(0.062, 0.720, 0.260, c=0.012, cz=-0.025))

# Rear bumper bar (dark)
write_stl("amr_bumper_rear.stl",
          chamfered_box(0.062, 0.720, 0.260, c=0.012, cz=-0.025))

# Drive wheel (axis along Z — matches joint rpy="-π/2 0 0")
write_stl("amr_wheel.stl",
          wheel_mesh(r=0.170, w=0.080, segs=48, tread_n=20))

# Central identification mast
write_stl("amr_mast.stl",
          frustum(r_top=0.048, r_bot=0.062, h=0.900, segs=12, cz=0.721))

# 360° LiDAR sensor ring (torus)
write_stl("amr_sensor_ring.stl",
          torus(R=0.120, r=0.050, seg_M=36, seg_m=14, cz=0.721))

# Safety beacon cylinder
write_stl("amr_beacon.stl",
          cylinder(r=0.068, h=0.200, segs=20, cz=1.281))

# Safety beacon dome (sphere)
write_stl("amr_dome.stl",
          sphere(r=0.090, segs=24, cz=1.476))

print()
print("All meshes written to:", OUT)
