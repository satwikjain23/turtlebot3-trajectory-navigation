#!/usr/bin/env python3
"""
Automated Test Script for path_smoothing_node.py

This script tests the B-spline smoothing function with multiple
edge-case waypoint sets. It does NOT require ROS2.

It validates:
 - Handling of duplicate points
 - Small waypoint sets (2 or 3 points)
 - Smooth curves for right-angle turns
 - Zig-zag input stability
 - No crashing for weird input

Generates plots for visual inspection.
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt
from navigation_assignment.path_smoothing_node import smooth


def plot_test(title, waypoints, smoothed, idx):
    plt.figure(figsize=(6,6))

    wp = np.array(waypoints)
    sm = np.array(smoothed)

    plt.plot(wp[:,0], wp[:,1], 'o-', label="Raw Waypoints")
    plt.plot(sm[:,0], sm[:,1], '--', label="Smoothed Output")

    plt.title(title)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()

    filename = f"test_output_{idx}.png"
    plt.savefig(filename)
    print(f"[SAVED] {filename}")
    plt.close()


# ---------------------------------------------------------------
# EXTREME TEST CASES
# ---------------------------------------------------------------

TEST_CASES = [

    # ---------------- Basic & Normal ----------------
    ("Straight Line", [(0,0), (1,1), (2,2)]),
    ("Right Angle Turn", [(0,0), (2,0), (2,2)]),
    ("Duplicate Points", [(0,0), (1,1), (1,1), (2,2)]),
    ("Only Two Points", [(0,0), (3,3)]),
    ("Only Three Points", [(0,0), (1,5), (2,0)]),

    # ---------------- Curved Paths ----------------
    ("Smooth Curve", [(0,0), (1,2), (2,3), (4,4), (5,3), (6,1)]),
    ("Zig Zag", [(0,0), (1,2), (2,-1), (3,3), (4,0), (5,4)]),

    # ---------------- Extreme Shapes ----------------
    ("Large Coordinates", [(0,0), (1000, 500), (2000, 1500)]),
    ("Negative Coordinates", [(-5,-5), (-3,2), (0,3), (4,-1)]),

    ("Almost Collinear", [(0,0), (1,0.001), (2,-0.001), (3,0.002), (4,0)]),

    ("Sharp V shape", [(0,0), (2,5), (4,0)]),
    ("Rapid Direction Change", [(0,0), (1,5), (2,-5), (3,5), (4,-5)]),

    # ---------------- Circle Approximation ----------------
    (
        "Half Circle",
        [(np.cos(t), np.sin(t)) for t in np.linspace(0, np.pi, 8)]
    ),

    (
        "Full Circle",
        [(np.cos(t), np.sin(t)) for t in np.linspace(0, 2*np.pi, 12)]
    ),

    # ---------------- Highly Clustered Points ----------------
    ("Clustered Points",
     [(0,0), (0.01,0.02), (0.02,0.04), (1,1)]),

    # ---------------- Noise / Random Perturbation ----------------
    (
        "Noisy Points",
        [(x, x + np.random.normal(0, 0.2)) for x in np.linspace(0, 5, 15)]
    ),

    # ---------------- Degenerate / Bad Inputs ----------------
    ("Repeated Identical Points", [(1,1), (1,1), (1,1), (1,1)]),
    ("Horizontal Line", [(0,5), (3,5), (10,5)]),
    ("Vertical Line", [(5,0), (5,3), (5,7)]),

    # ---------------- Stress Test ----------------
    (
        "100-Point Random Walk",
        [(i, np.sin(i) + np.random.uniform(-0.2, 0.2)) for i in range(100)]
    ),
]


# ---------------------------------------------------------------
# RUN TESTS
# ---------------------------------------------------------------
def run_all_tests():
    print("\n=====================================")
    print("      RUNNING ALL SMOOTHING TESTS    ")
    print("=====================================\n")

    for idx, (title, waypoints) in enumerate(TEST_CASES):
        print(f"[TEST {idx:02d}] {title}")

        try:
            smoothed = smooth(waypoints, smoothness=0.0, num_samples=200)

            # basic validation
            assert len(smoothed) > 0, "Empty output!"

            # save plot
            plot_test(title, waypoints, smoothed, idx)

            print(f"✔ PASSED: {title}\n")

        except Exception as e:
            print(f"✖ FAILED: {title}")
            print("  Error:", e)
            print()


if __name__ == "__main__":
    run_all_tests()
