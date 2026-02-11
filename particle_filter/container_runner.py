# particle_filter/container_runner.py (MODIFIED)

import argparse
import time
import os
import pandas as pd
import numpy as np
from particle_filter.measure_cpu import CPUSampler
from particle_filter.particle_filter_offline import ParticleFilterOffline

def load_dataset_from_csv(lidar_csv, gt_traj_csv, max_steps=None):
    """
    Load dataset from CSV files.
    
    Args:
        lidar_csv: Path to LiDAR CSV
        gt_traj_csv: Path to ground truth trajectory CSV
        max_steps: Maximum number of steps to load
    
    Returns:
        List of (pose, odom, scan) tuples
    """
    print("Loading dataset...")
    
    # Load ground truth trajectory
    gt_df = pd.read_csv(gt_traj_csv, names=['x', 'y', 'theta'])
    
    # Compute odometry deltas
    gt_df['dx'] = gt_df['x'].diff().fillna(0)
    gt_df['dy'] = gt_df['y'].diff().fillna(0)
    gt_df['dtheta'] = gt_df['theta'].diff().fillna(0)
    
    # Load LiDAR data
    lidar_df = pd.read_csv(lidar_csv)
    
    # Get range columns
    range_cols = [col for col in lidar_df.columns if col.startswith('field.ranges')]
    
    # Synchronize by row count
    n_rows = min(len(gt_df), len(lidar_df))
    if max_steps:
        n_rows = min(n_rows, max_steps)
    
    print(f"Loaded {n_rows} timesteps")
    
    # Build trajectory
    trajectory = []
    for i in range(n_rows):
        # Ground truth pose
        pose = (gt_df.iloc[i]['x'], gt_df.iloc[i]['y'], gt_df.iloc[i]['theta'])
        
        # Odometry delta
        odom = (gt_df.iloc[i]['dx'], gt_df.iloc[i]['dy'], gt_df.iloc[i]['dtheta'])
        
        # LiDAR scan (downsample and clean)
        ranges = lidar_df.iloc[i][range_cols].values
        # Replace NaN with max range
        ranges = np.nan_to_num(ranges, nan=5.6)
        # Downsample every 10 points
        scan = ranges[::10].astype(np.float32)
        
        trajectory.append((pose, odom, scan))
    
    return trajectory

def run_offline_test(pf, trajectory, out_csv):
    """
    Run particle filter on pre-loaded trajectory.
    
    Args:
        pf: ParticleFilterOffline instance
        trajectory: List of (pose, odom, scan) tuples
        out_csv: Output CSV path for results
    """
    print(f"Running particle filter on {len(trajectory)} steps...")
    
    rows = []
    t0 = time.time()
    
    for i, (pose, odom, scan) in enumerate(trajectory):
        # Run particle filter
        pf.step(odom, scan)
        
        # Get estimate
        est = pf.estimate()
        
        # Record results
        rows.append({
            "step": i,
            "time": time.time() - t0,
            "ground_x": pose[0],
            "ground_y": pose[1],
            "ground_th": pose[2],
            "est_x": est[0],
            "est_y": est[1],
            "est_th": est[2]
        })
        
        # Progress indicator
        if (i + 1) % 100 == 0:
            print(f"Processed {i+1}/{len(trajectory)} steps")
    
    elapsed = time.time() - t0
    print(f"Completed in {elapsed:.2f}s ({len(trajectory)/elapsed:.1f} Hz)")
    
    # Write results to CSV
    parent = os.path.dirname(out_csv)
    if parent and not os.path.exists(parent):
        os.makedirs(parent, exist_ok=True)
    
    import csv
    keys = ["step", "time", "ground_x", "ground_y", "ground_th", "est_x", "est_y", "est_th"]
    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)
    
    print(f"Results written to {out_csv}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar_csv", type=str, required=True, help="Path to LiDAR CSV file")
    parser.add_argument("--gt_traj_csv", type=str, required=True, help="Path to ground truth trajectory CSV")
    parser.add_argument("--map_file", type=str, default="test_map.pkl", help="Path to map pickle file")
    parser.add_argument("--particles", type=int, default=500, help="Number of particles")
    parser.add_argument("--steps", type=int, default=None, help="Max number of steps (None = all)")
    parser.add_argument("--out", type=str, default="pf_results.csv", help="Output CSV path")
    parser.add_argument("--cpu_csv", type=str, default="cpu_samples.csv", help="CPU samples CSV path")
    parser.add_argument("--sample_interval", type=float, default=0.05, help="CPU sample interval (s)")
    args = parser.parse_args()
    
    # Load dataset
    trajectory = load_dataset_from_csv(args.lidar_csv, args.gt_traj_csv, args.steps)
    
    # Initialize particle filter
    print(f"Initializing particle filter with {args.particles} particles...")
    pf = ParticleFilterOffline(n_particles=args.particles, map_file=args.map_file)
    
    # Start CPU monitoring
    sampler = CPUSampler(sample_interval=args.sample_interval, target_pid=os.getpid())
    sampler.start()
    print("CPU monitoring started")
    
    # Run test
    run_offline_test(pf, trajectory, args.out)
    
    # Stop CPU monitoring
    sampler.stop()
    sampler.join(timeout=2.0)
    sampler.to_csv(args.cpu_csv)
    print(f"CPU samples written to {args.cpu_csv}")
    
    return 0

if __name__ == "__main__":
    raise SystemExit(main())