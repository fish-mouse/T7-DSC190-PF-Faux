"""
Standalone benchmark for range_libc on Raspberry Pi 5 (CPU-only).
Tests all available ray casting methods and measures performance.
No ROS2 required - runs directly with Python.
"""
import numpy as np
import range_libc
import time
import os
import csv

def create_test_map(width=200, height=200):
    """Create a simple test occupancy grid (box with walls)."""
    grid = np.zeros((height, width), dtype=np.bool_)
    # walls around the edges
    grid[0, :] = True
    grid[-1, :] = True
    grid[:, 0] = True
    grid[:, -1] = True
    # some internal obstacles
    grid[50:60, 80:120] = True
    grid[100:110, 40:80] = True
    grid[140:150, 100:160] = True
    return grid

def benchmark_method(name, range_method, num_particles, num_rays, num_iterations):
    """Benchmark a single ray casting method."""
    # Create random queries (x, y, theta)
    queries = np.zeros((num_particles, 3), dtype=np.float32)
    queries[:, 0] = np.random.uniform(10, 190, num_particles)  # x
    queries[:, 1] = np.random.uniform(10, 190, num_particles)  # y
    queries[:, 2] = np.random.uniform(0, 2 * np.pi, num_particles)  # theta

    # Create angles for repeat_angles method
    angles = np.linspace(-1.5, 1.5, num_rays).astype(np.float32)
    ranges_out = np.zeros(num_particles * num_rays, dtype=np.float32)

    # Warm up
    range_method.calc_range_repeat_angles(queries, angles, ranges_out)

    # Benchmark
    times = []
    for _ in range(num_iterations):
        t_start = time.time()
        range_method.calc_range_repeat_angles(queries, angles, ranges_out)
        t_end = time.time()
        times.append(t_end - t_start)

    avg_time = np.mean(times)
    total_rays = num_particles * num_rays
    rays_per_sec = total_rays / avg_time

    print(f"  {name}:")
    print(f"    Avg time per iteration: {avg_time*1000:.2f} ms")
    print(f"    Total rays per call:    {total_rays:,}")
    print(f"    Rays per second:        {rays_per_sec:,.0f}")
    print(f"    Suitable for {int(1.0/avg_time)} Hz update rate")
    print()

    return {
        "method": name,
        "avg_time_ms": round(avg_time * 1000, 2),
        "rays_per_sec": int(rays_per_sec),
        "hz": int(1.0 / avg_time),
        "total_rays": total_rays,
    }

def main():
    print("=" * 60)
    print("  RANGE_LIBC BENCHMARK - CPU ONLY (No CUDA)")
    print("  Testing particle filter ray casting performance")
    print("=" * 60)
    print()

    # Parameters
    num_particles = 4000   # same as config/localize.yaml
    num_rays = 60          # 1080 / 18 (angle_step)
    num_iterations = 50
    max_range = 200
    theta_disc = 112

    print(f"Config: {num_particles} particles, {num_rays} rays, {num_iterations} iterations")
    print(f"Total rays per update: {num_particles * num_rays:,}")
    print()

    # Create test map
    grid = create_test_map()
    omap = range_libc.PyOMap(grid)

    # Test each CPU method
    results = []

    # 1. Bresenham's Line (slowest but simplest)
    print("Testing Bresenham's Line (BL)...")
    bl = range_libc.PyBresenhamsLine(omap, max_range)
    results.append(benchmark_method("BL", bl, num_particles, num_rays, num_iterations))

    # 2. Ray Marching (CPU)
    print("Testing Ray Marching (RM)...")
    rm = range_libc.PyRayMarching(omap, max_range)
    results.append(benchmark_method("RM", rm, num_particles, num_rays, num_iterations))

    # 3. CDDT (Corey Walsh's recommendation for your setup)
    print("Testing CDDT (Recommended by author)...")
    cddt = range_libc.PyCDDTCast(omap, max_range, theta_disc)
    results.append(benchmark_method("CDDT", cddt, num_particles, num_rays, num_iterations))

    # 4. PCDDT (Pruned CDDT - better for static maps)
    print("Testing PCDDT (Pruned CDDT - best for static indoor maps)...")
    pcddt = range_libc.PyCDDTCast(omap, max_range, theta_disc)
    pcddt.prune()
    results.append(benchmark_method("PCDDT", pcddt, num_particles, num_rays, num_iterations))

    # 5. Giant Lookup Table (fastest but uses most memory)
    print("Testing GLT (Giant Lookup Table)...")
    glt = range_libc.PyGiantLUTCast(omap, max_range, theta_disc)
    results.append(benchmark_method("GLT", glt, num_particles, num_rays, num_iterations))

    # Summary
    print("=" * 60)
    print("  SUMMARY")
    print("=" * 60)
    print(f"  {'Method':<8} {'Time (ms)':<12} {'Rays/sec':<15} {'Update Hz'}")
    print(f"  {'-'*8} {'-'*12} {'-'*15} {'-'*10}")
    for r in results:
        print(f"  {r['method']:<8} {r['avg_time_ms']:<12} {r['rays_per_sec']:<15,} {r['hz']}")
    print()

    # Check if GPU method exists (it will, but can't be used)
    try:
        rmgpu = range_libc.PyRayMarchingGPU(omap, max_range)
        print("WARNING: PyRayMarchingGPU loaded but requires CUDA (will crash if used)")
    except Exception as e:
        print(f"PyRayMarchingGPU not available (expected): {e}")

    # Save results to CSV
    csv_path = "/data/range_libc_benchmark.csv"
    try:
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["method", "avg_time_ms", "rays_per_sec", "hz", "total_rays"])
            writer.writeheader()
            writer.writerows(results)
        print(f"\nResults saved to {csv_path}")
    except Exception:
        print("\n(Could not save CSV - run with -v flag to mount /data)")

    print("\nDone!")

if __name__ == "__main__":
    main()
