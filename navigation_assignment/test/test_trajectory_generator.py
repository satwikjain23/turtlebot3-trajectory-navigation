#!/usr/bin/env python3
"""
Automated test suite for trajectory generation logic.

This DOES NOT import or instantiate the ROS node.
We directly test the underlying mathematics:
    - distance → time conversion
    - monotonic timestamps
    - correct path length
    - edge cases and stress tests
"""

import numpy as np
import matplotlib.pyplot as plt


# ---------------------------------------------------------
# PURE FUNCTION FOR TESTING (NO ROS)
# ---------------------------------------------------------
def compute_trajectory_from_points(points, velocity=0.2):
    traj = []
    prev_x, prev_y = points[0]
    t = 0.0

    for (x, y) in points:
        dist = np.hypot(x - prev_x, y - prev_y)
        dt = dist / velocity
        t += dt
        traj.append((x, y, t))

        prev_x, prev_y = x, y

    return traj


# ---------------------------------------------------------
# TEST CASES
# ---------------------------------------------------------
TEST_CASES = [

    ("Straight Line", [(0,0), (1,1), (2,2)]),

    ("Two Points Only", [(0, 0), (3, 4)]),  # 5m distance

    ("Duplicate Points", [(1,1), (1,1), (1,1), (2,2)]),

    ("Negative Coordinates", [(-2,-3), (-1,-1), (0,0), (2,1)]),

    ("Large Distances", [(0,0), (100,100), (200,200)]),

    ("Noise Zigzag",
     [(x, np.sin(x) + np.random.uniform(-0.2,0.2)) for x in np.linspace(0, 5, 12)]
    ),

    ("Vertical", [(5,0), (5,4), (5,10)]),

    ("Horizontal", [(0,5), (5,5), (10,5)]),

    ("Random Walk",
     [(i, i + np.random.uniform(-3,3)) for i in range(20)]
    ),

    ("Stress Test 1000 Points",
     [(i, np.sin(i)) for i in range(1000)]
    ),
]


# ---------------------------------------------------------
# RUN ALL TESTS
# ---------------------------------------------------------
def run_trajectory_tests():
    print("\n===================================")
    print("   Running Trajectory Generator Tests")
    print("===================================\n")

    for idx, (name, waypoints) in enumerate(TEST_CASES):
        print(f"[TEST {idx:02d}] {name}")

        try:
            traj = compute_trajectory_from_points(waypoints)

            # ---- Test 1: Length matches ----
            assert len(traj) == len(waypoints), "Trajectory size mismatch!"

            # ---- Test 2: Timestamps monotonic ----
            times = [t for (_,_,t) in traj]
            assert all(times[i] <= times[i+1] for i in range(len(times)-1)), \
                "Time is not increasing!"

            # ---- Test 3: Distance-time correctness ----
            v = 0.2
            for i in range(1, len(traj)):
                x1, y1, t1 = traj[i-1]
                x2, y2, t2 = traj[i]
                dist = np.hypot(x2-x1, y2-y1)
                dt = t2 - t1

                assert abs(dist - v*dt) < 1e-3, "Distance/time mismatch!"

            # ---- Visual Plot ----
            xs = [p[0] for p in traj]
            ys = [p[1] for p in traj]

            plt.figure()
            plt.plot(xs, ys, marker='o')
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title(f"Trajectory Test: {name}")

            filename = f"trajectory_test_{idx}.png"
            plt.savefig(filename)
            plt.close()

            print(f"✔ PASSED: {name}\n")

        except Exception as e:
            print(f"✖ FAILED: {name}")
            print("  Error:", e, "\n")


if __name__ == "__main__":
    run_trajectory_tests()

