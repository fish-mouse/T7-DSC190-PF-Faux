# Container-friendly runner. Supports inproc and subprocess modes.
import argparse
import importlib
import subprocess
import time
import os
from particle_filter.measure_cpu import CPUSampler
from particle_filter.fake_data import FakeWorld

CANDIDATES = [
    "particle_filter",
    "particle_filter.particle_filter",
    "particle_filter.filter",
    "particle_filter.pf",
]

def find_and_instantiate(n_particles):
    for name in CANDIDATES:
        try:
            mod = importlib.import_module(name)
        except Exception:
            continue
        cls = None
        if hasattr(mod, "ParticleFilter"):
            cls = getattr(mod, "ParticleFilter")
        elif hasattr(mod, "ParticleFiler"):
            cls = getattr(mod, "ParticleFiler")
        if cls:
            try:
                return cls(n_particles=n_particles)
            except TypeError:
                pass
            try:
                return cls(n_particles)
            except TypeError:
                pass
            try:
                return cls()
            except Exception:
                pass
    raise RuntimeError("Could not import/instantiate particle filter from candidates: " + ", ".join(CANDIDATES))

def default_control_fn(i):
    return (0.2, 0.05)

def run_inproc(pf, particles, steps, out):
    world = FakeWorld()
    traj = world.generate_trajectory(steps, default_control_fn)
    rows = []
    t0 = time.time()
    for i, (pose, odom, meas) in enumerate(traj):
        try:
            if hasattr(pf, "step"):
                pf.step(odom, meas)
            else:
                if hasattr(pf, "predict"):
                    pf.predict(odom)
                if hasattr(pf, "update"):
                    pf.update(meas)
        except Exception:
            pass
        est = None
        if hasattr(pf, "estimate"):
            try:
                est = pf.estimate()
            except Exception:
                est = None
        rows.append({
            "step": i,
            "time": time.time() - t0,
            "ground_x": pose[0],
            "ground_y": pose[1],
            "ground_th": pose[2],
            "est_x": est[0] if est else "",
            "est_y": est[1] if est else "",
        })
    # write run CSV
    parent = os.path.dirname(out)
    if parent and not os.path.exists(parent):
        os.makedirs(parent, exist_ok=True)
    import csv
    keys = ["step","time","ground_x","ground_y","ground_th","est_x","est_y"]
    with open(out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)

def run_subprocess(cmd):
    p = subprocess.Popen(cmd, shell=True)
    return p

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["inproc","subprocess"], default="inproc")
    parser.add_argument("--subcmd", type=str, default="")
    parser.add_argument("--particles", type=int, default=500)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--out", type=str, default="/data/pf_run.csv")
    parser.add_argument("--cpu_csv", type=str, default="/data/cpu_samples.csv")
    parser.add_argument("--sample_interval", type=float, default=0.05)
    args = parser.parse_args()

    if args.mode == "inproc":
        try:
            pf = find_and_instantiate(args.particles)
        except Exception as e:
            print("Inproc instantiation failed:", e)
            return 2
        sampler = CPUSampler(sample_interval=args.sample_interval, target_pid=os.getpid())
        sampler.start()
        run_inproc(pf, args.particles, args.steps, args.out)
        sampler.stop()
        sampler.join(timeout=2.0)
        sampler.to_csv(args.cpu_csv)
        print("Finished inproc run. outputs:", args.out, args.cpu_csv)
        return 0
    else:
        if not args.subcmd:
            print("subprocess mode requires --subcmd")
            return 2
        child = run_subprocess(args.subcmd)
        sampler = CPUSampler(sample_interval=args.sample_interval, target_pid=child.pid)
        sampler.start()
        try:
            ret = child.wait()
        except KeyboardInterrupt:
            child.terminate()
            child.wait()
            ret = -1
        sampler.stop()
        sampler.join(timeout=2.0)
        sampler.to_csv(args.cpu_csv)
        print("Finished subprocess run. child return:", ret, "cpu_csv:", args.cpu_csv)
        return ret

if __name__ == "__main__":
    raise SystemExit(main())