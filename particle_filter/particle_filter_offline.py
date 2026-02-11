# particle_filter/particle_filter_offline.py

"""
Offline version of particle_filter.py for CPU benchmarking.
Removes ROS2 dependencies, loads data from CSV.
"""
import pandas as pd
import numpy as np
import range_libc
import time
import pickle
from threading import Lock

class ParticleFilterOffline:
    """Monte Carlo Localization - Offline testing version."""
    
    def __init__(self, n_particles=500, map_file='test_map.pkl'):
        self.MAX_PARTICLES = n_particles
        self.ANGLE_STEP = 10  # Downsample lidar every 10 points
        self.MAX_RANGE_METERS = 5.6
        self.THETA_DISCRETIZATION = 112
        
        # Sensor model constants
        self.Z_SHORT = 0.01
        self.Z_MAX = 0.07
        self.Z_RAND = 0.12
        self.Z_HIT = 0.80
        self.SIGMA_HIT = 8.0
        
        # Motion model constants
        self.MOTION_DISPERSION_X = 0.05
        self.MOTION_DISPERSION_Y = 0.025
        self.MOTION_DISPERSION_THETA = 0.25
        
        # State variables
        self.particles = np.zeros((self.MAX_PARTICLES, 3))
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)
        self.particle_indices = np.arange(self.MAX_PARTICLES)
        self.inferred_pose = None
        self.state_lock = Lock()
        
        # Cache for motion model
        self.local_deltas = np.zeros((self.MAX_PARTICLES, 3))
        
        # Cache for sensor model
        self.queries = None
        self.ranges = None
        self.tiled_angles = None
        self.first_sensor_update = True
        
        # Load map and initialize range method
        self._load_map(map_file)
        self._precompute_sensor_model()
        self._initialize_particles()
        
        print(f"Initialized offline particle filter with {n_particles} particles")
    
    def _load_map(self, map_file):
        """Load map from pickle file."""
        with open(map_file, 'rb') as f:
            map_info_dict = pickle.load(f)
        
        # Extract map parameters
        resolution = map_info_dict['resolution']
        width = map_info_dict['width']
        height = map_info_dict['height']
        origin_x = map_info_dict['origin_x']
        origin_y = map_info_dict['origin_y']
        
        # Convert to numpy array (height x width)
        map_data = np.array(map_info_dict['data'], dtype=np.int8).reshape((height, width))
        
        # Create range_libc PyOMap (just pass the map data)
        self.omap = range_libc.PyOMap(map_data)
        
        # Store map info as simple object
        class MapInfo:
            pass
        
        self.map_info = MapInfo()
        self.map_info.resolution = resolution
        self.map_info.width = width
        self.map_info.height = height
        self.map_info.origin_x = origin_x
        self.map_info.origin_y = origin_y
        
        self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS / resolution)
        
        # Initialize range method (CDDT for speed)
        try:
            self.range_method = range_libc.PyCDDTCast(
                self.omap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION
            )
            print("Initialized CDDT range method")
        except Exception as e:
            print(f"CDDT failed: {e}, trying Bresenham...")
            # Fallback to simpler method if CDDT fails
            try:
                self.range_method = range_libc.PyBresenhamsLine(self.omap, self.MAX_RANGE_PX)
                print("Initialized Bresenham's line range method")
            except Exception as e2:
                print(f"Bresenham also failed: {e2}")
                # Last resort: use simple ray marching
                self.range_method = range_libc.PyRayMarching(self.omap, self.MAX_RANGE_PX)
                print("Initialized Ray Marching range method")
    
    def _initialize_particles(self):
        """Initialize particles uniformly in free space."""
        # Simple initialization: uniform random in map bounds
        map_width_m = self.map_info.width * self.map_info.resolution
        map_height_m = self.map_info.height * self.map_info.resolution
        
        self.particles[:, 0] = np.random.uniform(0.5, map_width_m - 0.5, self.MAX_PARTICLES)
        self.particles[:, 1] = np.random.uniform(0.5, map_height_m - 0.5, self.MAX_PARTICLES)
        self.particles[:, 2] = np.random.uniform(-np.pi, np.pi, self.MAX_PARTICLES)
        self.weights[:] = 1.0 / self.MAX_PARTICLES
    
    def _precompute_sensor_model(self):
        """Generate sensor model lookup table."""
        table_width = int(self.MAX_RANGE_PX) + 1
        self.sensor_model_table = np.zeros((table_width, table_width))
        
        for d in range(table_width):
            norm = 0.0
            for r in range(table_width):
                prob = 0.0
                z = float(r - d)
                
                # Hit probability (Gaussian)
                prob += self.Z_HIT * np.exp(-(z*z)/(2.0*self.SIGMA_HIT*self.SIGMA_HIT)) / \
                        (self.SIGMA_HIT * np.sqrt(2.0*np.pi))
                
                # Short reading probability
                if r < d:
                    prob += 2.0 * self.Z_SHORT * (d - r) / float(d) if d > 0 else 0
                
                # Max range probability
                if int(r) == int(self.MAX_RANGE_PX):
                    prob += self.Z_MAX
                
                # Random measurement probability
                if r < int(self.MAX_RANGE_PX):
                    prob += self.Z_RAND * 1.0 / float(self.MAX_RANGE_PX)
                
                norm += prob
                self.sensor_model_table[int(r), int(d)] = prob
            
            # Normalize
            if norm > 0:
                self.sensor_model_table[:, int(d)] /= norm
        
        # Upload to range_libc
        self.range_method.set_sensor_model(self.sensor_model_table)
    
    def motion_model(self, proposal_dist, action):
        """Apply motion model with noise."""
        cosines = np.cos(proposal_dist[:, 2])
        sines = np.sin(proposal_dist[:, 2])
        
        self.local_deltas[:, 0] = cosines * action[0] - sines * action[1]
        self.local_deltas[:, 1] = sines * action[0] + cosines * action[1]
        self.local_deltas[:, 2] = action[2]
        
        proposal_dist[:, :] += self.local_deltas
        proposal_dist[:, 0] += np.random.normal(0.0, self.MOTION_DISPERSION_X, self.MAX_PARTICLES)
        proposal_dist[:, 1] += np.random.normal(0.0, self.MOTION_DISPERSION_Y, self.MAX_PARTICLES)
        proposal_dist[:, 2] += np.random.normal(0.0, self.MOTION_DISPERSION_THETA, self.MAX_PARTICLES)
    
    def sensor_model(self, proposal_dist, observation):
        """Evaluate sensor model."""
        num_rays = len(observation)
        
        # Allocate buffers on first call
        if self.first_sensor_update:
            self.queries = np.zeros((self.MAX_PARTICLES, 3), dtype=np.float32)
            self.ranges = np.zeros(num_rays * self.MAX_PARTICLES, dtype=np.float32)
            self.tiled_angles = np.tile(
                np.linspace(-2.09, 2.09, num_rays), self.MAX_PARTICLES
            )
            self.first_sensor_update = False
        
        # Compute expected ranges for all particles
        self.queries[:, :] = proposal_dist[:, :]
        
        try:
            # Use range_libc for fast raycasting
            angles = np.linspace(-2.09, 2.09, num_rays).astype(np.float32)
            self.range_method.calc_range_repeat_angles(self.queries, angles, self.ranges)
            
            # Evaluate sensor model
            self.range_method.eval_sensor_model(
                observation, self.ranges, self.weights, num_rays, self.MAX_PARTICLES
            )
            
            # Apply squash factor
            np.power(self.weights, 0.5, self.weights)
        except Exception as e:
            # Fallback: uniform weights
            print(f"Sensor model error: {e}")
            self.weights[:] = 1.0 / self.MAX_PARTICLES
    
    def step(self, odom, scan):
        """
        Single MCL update step.
        
        Args:
            odom: Tuple (dx, dy, dtheta) - odometry delta
            scan: Array of LiDAR ranges
        """
        self.state_lock.acquire()
        
        # Resample
        proposal_indices = np.random.choice(
            self.particle_indices, self.MAX_PARTICLES, p=self.weights
        )
        proposal_distribution = self.particles[proposal_indices, :]
        
        # Motion model
        self.motion_model(proposal_distribution, odom)
        
        # Sensor model
        self.sensor_model(proposal_distribution, scan)
        
        # Normalize weights
        self.weights /= np.sum(self.weights)
        
        # Update particles
        self.particles = proposal_distribution
        
        # Compute estimate
        self.inferred_pose = np.average(self.particles, weights=self.weights, axis=0)
        
        self.state_lock.release()
    
    def estimate(self):
        """Return current pose estimate."""
        return self.inferred_pose if self.inferred_pose is not None else np.array([0, 0, 0])