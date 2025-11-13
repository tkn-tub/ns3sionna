import argparse
import math

# ZMQ, PB
import zmq
import message_pb2

# Sionna
import os

from commons import *
from sionna_utils import compute_coherence_time

gpu_num = 0 # Use "" to use the CPU
os.environ["CUDA_VISIBLE_DEVICES"] = f"{gpu_num}"
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

import sionna
import tensorflow as tf
from tensorflow.python.tools.optimize_for_inference_lib import node_from_map, node_name_from_input
import mitsuba as mi
import numpy as np
import warnings
import time

gpus = tf.config.list_physical_devices('GPU')
if gpus:
    try:
        tf.config.experimental.set_memory_growth(gpus[0], True)
    except RuntimeError as e:
        print(e)
tf.get_logger().setLevel('ERROR')

from sionna.rt import load_scene, Transmitter, Receiver, PlanarArray, Camera
from sionna.channel import cir_to_ofdm_channel, subcarrier_frequencies
from sionna.rt.antenna import iso_pattern

class CacheEntry:
    def __init__(self, sim_time, ttl, value):
        self.sim_time = sim_time
        self.ttl = ttl
        self.value = value

    def debug(self):
        return str(self.sim_time) + "/" + str(self.ttl) + "/..."

class SionnaEnv:
    """
    This class represents a Sionna environment where the node placement, mobility is controlled from
    outside using ZMQ messages.

    author: Pilz, Zubow
    """

    def __init__(self, rt_calc_diffraction, rt_max_depth=5, rt_max_parallel_links=32, est_csi=True, VERBOSE=True):
        self.rt_calc_diffraction = rt_calc_diffraction
        self.rt_max_depth = rt_max_depth
        self.rt_max_parallel_links = rt_max_parallel_links
        self.est_csi = est_csi
        self.VERBOSE = VERBOSE
        self.node_info_dict = {}
        self.last_placed_nodes = [] # name of TX/RX placed during last channel computation
        self.pos_velo_cache = dict()


    def store_simulation_info(self, simulation_info):
        """
        Prepares Sionna by setting the simulation parameter
        """
        # global gpus

        # Load the sionna scene
        filepath = "./../models/" + simulation_info.scene_fname
        self.scene = load_scene(filepath)
        self.mode = simulation_info.mode

        if simulation_info.sub_mode > -1:
            # set from ns-3
            self.sub_mode = simulation_info.sub_mode
        else:
            # use value set by sionna
            self.sub_mode = self.rt_max_parallel_links

        print(f'Operating in mode: {self.mode}, sub_mode: {self.sub_mode}')

        # Load the mitsuba scene used by the mobility model
        # Mobility model uses mitsuba SCALAR variant
        mi.set_variant('scalar_rgb')
        self.mi_scene = mi.load_file(filepath, parallel=False)

        # Set the mitsuba variant back to how Sionna configures it
        if len(gpus) > 0:
            mi.set_variant('cuda_ad_rgb')
        else:
            mi.set_variant('llvm_ad_rgb')

        # SISO mode only
        # Configure antenna array for all transmitters
        self.scene.tx_array = PlanarArray(num_rows=1,
                                     num_cols=1,
                                     vertical_spacing=0.5,
                                     horizontal_spacing=0.5,
                                     pattern=iso_pattern)

        # Configure antenna array for all receivers
        self.scene.rx_array = PlanarArray(num_rows=1,
                                     num_cols=1,
                                     vertical_spacing=0.5,
                                     horizontal_spacing=0.5,
                                     pattern=iso_pattern)
        print(f'Scenario: {simulation_info.scene_fname}')
        print(f'Params: F0={simulation_info.frequency}MHz, BW={simulation_info.channel_bw}MHz, '
              f'FFT={simulation_info.fft_size}, df={simulation_info.subcarrier_spacing}Hz, '
              f'minTc={simulation_info.min_coherence_time_ms}ms')

        # Set scene parameters
        self.scene.frequency = simulation_info.frequency * 1e6
        self.scene.channel_bw = simulation_info.channel_bw * 1e6 # max channel bandwidth
        self.scene.fft_size = simulation_info.fft_size  # max FFT size
        self.scene.min_coherence_time_ms = simulation_info.min_coherence_time_ms # min Tc
        self.scene.subcarrier_spacing = simulation_info.subcarrier_spacing # in Hz

        # If set to False, ray tracing will be done per antenna element (slower for large arrays)
        self.scene.synthetic_array = True

        # Set the random seed for reproducibility
        np.random.seed(simulation_info.seed)
        tf.random.set_seed(simulation_info.seed)

        # Store information about each node
        for node_info in simulation_info.nodes:
            if (node_info.HasField("constant_position_model")):
                position = node_info.constant_position_model.position

                self.node_info_dict[node_info.id] = {
                    "model": "Constant Position",
                    "position": [position.x, position.y, position.z],
                    "speed": ("Constant", 0.0)
                }

            elif (node_info.HasField("random_walk_model")):

                random_walk_model = node_info.random_walk_model
                position = random_walk_model.position

                mode = ()
                if random_walk_model.HasField("time_value"):
                    mode = ("Time", random_walk_model.time_value)
                elif random_walk_model.HasField("distance_value"):
                    mode = ("Distance", random_walk_model.distance_value)

                speed = ()
                if random_walk_model.speed.HasField("uniform"):
                    speed = ("Uniform", random_walk_model.speed.uniform.min, random_walk_model.speed.uniform.max)
                elif random_walk_model.speed.HasField("constant"):
                    speed = ("Constant", random_walk_model.speed.constant.value)
                elif random_walk_model.speed.HasField("normal"):
                    speed = ("Normal", random_walk_model.speed.normal.mean, random_walk_model.speed.normal.variance)

                direction = ()
                if random_walk_model.direction.HasField("uniform"):
                    direction = (
                    "Uniform", random_walk_model.direction.uniform.min, random_walk_model.direction.uniform.max)
                elif random_walk_model.direction.HasField("constant"):
                    direction = ("Constant", random_walk_model.direction.constant.value)
                elif random_walk_model.direction.HasField("normal"):
                    direction = (
                    "Normal", random_walk_model.direction.normal.mean, random_walk_model.direction.normal.variance)

                self.node_info_dict[node_info.id] = {
                    "model": "Random Walk",
                    "position": [position.x, position.y, position.z],
                    "velocity": [0.0, 0.0, 0.0],
                    "mode": mode,
                    "speed": speed,
                    "direction": direction,
                    "last update": 0,
                    "delay left": 0
                }

        # create cache for mode 3
        for node_id in list(self.node_info_dict.keys()):
            self.pos_velo_cache[node_id] = []

        # check mode compatibility
        if self.mode == 3 or self.mode == 2:
            # only constant speed model supported
            for node_id in list(self.node_info_dict.keys()):
                if self.node_info_dict[node_id]["speed"][0] != "Constant":
                    warnings.warn(
                        f"Only constant speed model is supported when using mode 2/3; switching to mode 1.",
                        UserWarning)
                    self.mode = 1

        # estimate the max speed in network for Tc
        max_v = 1e-4 # max coherence time of 100s
        for rx_node in list(self.node_info_dict.keys()):
            max_v = max(max_v, self.node_info_dict[rx_node]["speed"][1])

        # coherence time in nanoseconds
        #self.chan_coh_time_mode23 = int(9 * 299792458 * 1e9 / (16 * np.pi * 2 * max_v * self.scene.frequency.numpy()))
        self.chan_coh_time_mode23 = compute_coherence_time(max_v, self.scene.frequency.numpy(), model='rappaport2')
        # consider minimum coherence time which is given in ms
        self.chan_coh_time_mode23 = min(self.chan_coh_time_mode23, self.scene.min_coherence_time_ms * 1e6)

        if self.mode == 3 or self.mode == 2:
            # worst case coherence time
            print("Running mode %d with Tc=%.2f ms" % (self.mode, self.chan_coh_time_mode23 / 1e6))

        num_nodes = len(self.node_info_dict)

        # how long to store computed position values from mobility
        self.max_pos_cache_age = max(1e9,  num_nodes * math.ceil(self.sub_mode / num_nodes) * self.chan_coh_time_mode23)

        if self.VERBOSE:
            print_simulation_info(simulation_info)


    def calculate_channel_state(self, channel_state_request, reply_wrapper):

        tx_node = channel_state_request.tx_node
        mand_rx_node = channel_state_request.rx_node # this rx node must be included in result set
        simulation_time = channel_state_request.time # in nanoseconds

        # remove all entries from cache
        self.remove_all_cached_entries(simulation_time)

        # Remove all last transmitter and receiver
        for node_name in self.last_placed_nodes:
            self.scene.remove(node_name)
        self.last_placed_nodes.clear()

        # Get all receiver IDs
        if self.mode == 1: # P2P
            all_rx_nodes = [mand_rx_node]
        else: # P2MP
            all_rx_nodes = list(self.node_info_dict.keys())
            all_rx_nodes.remove(tx_node)

        # number of channel calculations in look ahead
        if self.mode == 3:
            look_ahead = math.ceil(self.sub_mode / len(all_rx_nodes))
        else:
            look_ahead = 1

        #if self.VERBOSE:
        if self.mode == 1:
            print("Calc channel called:: %.6f: %d -> %d" % (simulation_time/1e9, tx_node, mand_rx_node))
        else: # mode 2, 3
            print("Calc channel called:: %.6f: %d -> %d, #MP=%d, LAH=%d, Tc=%.2f ms"
                      % (simulation_time/1e9, tx_node, mand_rx_node, len(all_rx_nodes), look_ahead, self.chan_coh_time_mode23/1e6))

        add_to_cache = dict()
        for node_id in list(self.node_info_dict.keys()):
            add_to_cache[node_id] = []

        tx_pos = {}
        tx_v = {}
        all_rx_pos = {}
        all_rx_v = {}
        # sim future node locations
        for future_id in range(look_ahead):
            future_simulation_time = int(simulation_time + future_id * self.chan_coh_time_mode23)
            tx_pos[future_id] = []
            tx_v[future_id] = []
            all_rx_pos[future_id] = []
            all_rx_v[future_id] = []

            # Get the current node positions and velocities
            tx_node_position, tx_node_velocity = self.get_position_and_velocity(tx_node, future_simulation_time)
            tx_pos[future_id] = tx_node_position
            tx_v[future_id] = tx_node_velocity
            ce = CacheEntry(future_simulation_time, self.chan_coh_time_mode23, (tx_node_position, tx_node_velocity))
            add_to_cache[tx_node].append(ce)

            # Create the transmitter
            tx_node_name = "tx" + str(future_id)
            tx = Transmitter(name=tx_node_name,
                             position=tx_node_position)

            # Add transmitter instance to scene
            self.scene.add(tx)
            self.last_placed_nodes.append(tx_node_name)

            # place all nodes as RX
            for rx_node in all_rx_nodes:

                if self.VERBOSE:
                    print_csi_request(future_simulation_time, tx_node, rx_node)

                # Get the current node positions and velocities
                rx_node_position, rx_node_velocity = self.get_position_and_velocity(rx_node, future_simulation_time)
                all_rx_pos[future_id].append(rx_node_position)
                all_rx_v[future_id].append(rx_node_velocity)

                ce = CacheEntry(future_simulation_time, self.chan_coh_time_mode23, (rx_node_position, rx_node_velocity))
                add_to_cache[rx_node].append(ce)

                rx_node_name = "rx" + str(rx_node) + "." + str(future_id)
                # Create the receiver
                rx = Receiver(name=rx_node_name,
                              position=rx_node_position)

                # Add receiver instance to scene
                self.scene.add(rx)
                self.last_placed_nodes.append(rx_node_name)

        # update pos cache
        for node_id in list(add_to_cache.keys()):
            for tmp in add_to_cache[node_id]:
                self.pos_velo_cache[node_id].append(tmp)

        # WiFi parameters
        subcarrier_spacing = self.scene.subcarrier_spacing #(self.scene.channel_bw / self.scene.fft_size)
        fft_size = self.scene.fft_size

        a, tau = 0, 0
        a_tau_set = False

        # Compute propagation paths
        paths = self.scene.compute_paths(max_depth=self.rt_max_depth,
                                    method="fibonacci",
                                    num_samples=1e6,
                                    los=True,
                                    reflection=True,
                                    diffraction=self.rt_calc_diffraction,
                                    scattering=False)

        has_paths = bool(paths.types.numpy().size)
        has_los_path = np.any(paths.types.numpy()[0] == 0)

        # If no LOS path was found, check again with different compute_paths parameters
        if not has_los_path:
            los_path = self.scene.compute_paths(max_depth=0,
                                           method="fibonacci",
                                           num_samples=1e6,
                                           los=True,
                                           reflection=False,
                                           diffraction=False,
                                           scattering=False)

            has_los_path = bool(los_path.types.numpy().size)

            if not has_paths and not has_los_path:
                raise SystemExit(
                    "Error: Propagation loss and propagation delay cannot be calculated because no propagation paths were found. "
                    "Make sure that the nodes are not spatially separated in the 3D model and check the parameters of the compute_path() function.")
            if has_los_path:
                # Disable normalization of delays for LOS path
                los_path.normalize_delays = False
                # Compute the channel impulse response for LOS path
                a, tau = los_path.cir()
                a_tau_set = True

        if has_paths:
            # Disable normalization of delays for paths
            paths.normalize_delays = False
            # Compute the channel impulse response for path
            a_paths, tau_paths = paths.cir()

            # Set a and tau
            if a_tau_set:
                a = tf.concat([a, a_paths], axis=5)
                tau = tf.concat([tau, tau_paths], axis=3)
            else:
                a, tau = a_paths, tau_paths

        # Compute the frequencies of subcarriers and center around carrier frequency
        frequencies = subcarrier_frequencies(num_subcarriers=fft_size,
                                             subcarrier_spacing=subcarrier_spacing)

        # Compute the frequency response of the channel at frequencies
        # tensor: [batch size, num_rx, num_rx_ant, num_tx, num_tx_ant, num_time_steps, fft_size]
        # absolute
        h_freq_raw = cir_to_ofdm_channel(frequencies=frequencies, a=a, tau=tau, normalize=False)
        # norm
        h_freq = cir_to_ofdm_channel(frequencies=frequencies, a=a, tau=tau, normalize=True)

        # ZMQ response
        chan_response = reply_wrapper.channel_state_response

        for future_id in range(look_ahead):
            future_simulation_time = int(simulation_time + future_id * self.chan_coh_time_mode23)
            # add new CSI
            csi = chan_response.csi.add()

            csi.start_time = future_simulation_time
            csi.end_time = int(future_simulation_time + self.chan_coh_time_mode23)
            # tx node info
            csi.tx_node.id = tx_node
            csi.tx_node.position.x = tx_pos[future_id][0]
            csi.tx_node.position.y = tx_pos[future_id][1]
            csi.tx_node.position.z = tx_pos[future_id][2]

            for lnk_id, rx_node in enumerate(all_rx_nodes):
                # compute the index for the rx nodes into tensor
                tf_index = future_id * len(all_rx_nodes) + lnk_id

                lnk_h_freq_raw = h_freq_raw.numpy()[:, tf_index, :, future_id, :, :, :]
                lnk_h_freq = h_freq.numpy()[:, tf_index, :, future_id, :, :, :]
                lnk_tau = tau.numpy()[:, tf_index, future_id, :]

                # Calculate propagation delay and propagation loss
                lnk_delay = int(round(np.min(lnk_tau[lnk_tau >= 0] * 1e9), 0))

                # see Parseval's theorem
                lnk_loss = float(-10 * np.log10(tf.reduce_mean(tf.abs(lnk_h_freq_raw) ** 2).numpy()))

                # the channel frequency response (CFR)
                lnk_csi = lnk_h_freq.flatten()

                if self.mode == 1 and self.sub_mode > 0:
                    # Calculate the time to live for the cache entry with the coherence time and the remaining times
                    # until the nodes change their walk direction
                    # If both nodes have the constant position model, the ttl is equal to one hour
                    tx_delay_left = 3.6e12
                    rx_delay_left = 3.6e12
                    if self.node_info_dict[tx_node]["model"] == "Random Walk":
                        tx_delay_left = self.node_info_dict[tx_node]["delay left"]
                    if self.node_info_dict[rx_node]["model"] == "Random Walk":
                        rx_delay_left = self.node_info_dict[rx_node]["delay left"]

                    lnk_delay_left = min(tx_delay_left, rx_delay_left)

                    lnk_ttl = int(lnk_delay_left)
                    lnk_v = np.linalg.norm(np.array(tx_v[future_id]) - np.array(all_rx_v[future_id][lnk_id]))

                    if lnk_v != 0:
                        # compute channel coherence time
                        lnk_ttl = int(
                            min(9 * 299792458 * 1e9 / (16 * np.pi * lnk_v * self.scene.frequency.numpy()), lnk_delay_left))

                    csi.end_time = int(future_simulation_time + lnk_ttl)

                #if self.VERBOSE:
                #    self.print_csi_response(simulation_time, tx_node, rx_node, tx_node_position, all_rx_pos[future_id][lnk_id], lnk_delay, lnk_loss, lnk_ttl)

                rx_node_info = csi.rx_nodes.add()
                rx_node_info.id = rx_node
                rx_node_info.position.x = all_rx_pos[future_id][lnk_id][0]
                rx_node_info.position.y = all_rx_pos[future_id][lnk_id][1]
                rx_node_info.position.z = all_rx_pos[future_id][lnk_id][2]
                rx_node_info.delay = lnk_delay
                rx_node_info.wb_loss = lnk_loss

                if self.est_csi:
                    # avoid rounding errors
                    frequencies = np.arange(-fft_size / 2, fft_size / 2, dtype=int) * subcarrier_spacing
                    rx_node_info.frequencies.extend(frequencies.tolist())
                    #rx_node_info.frequencies.extend(list(frequencies.numpy().astype(int)))
                    rx_node_info.csi_imag.extend(list(np.imag(lnk_csi)))
                    rx_node_info.csi_real.extend(list(np.real(lnk_csi)))

        #if self.VERBOSE:
        last_sim = simulation_time + (look_ahead - 1) * self.chan_coh_time_mode23
        print("Calc channel finished:: LAH: Twin=%.6f -> %.6f" % (simulation_time/1e9, last_sim/1e9))


    def get_value(self, random_variable):
        if random_variable[0] == "Uniform":
            return np.random.uniform(random_variable[1], random_variable[2])

        elif random_variable[0] == "Constant":
            return random_variable[1]

        elif random_variable[0] == "Normal":
            return np.random.normal(random_variable[1], np.sqrt(random_variable[2]))


    def walk(self, node_id, delay_left):

        self.node_info_dict[node_id]["last update"] += delay_left

        position = np.array(self.node_info_dict[node_id]["position"])
        velocity = np.array(self.node_info_dict[node_id]["velocity"])

        speed = np.linalg.norm(velocity)
        if speed == 0:
            return

        # Check if the next position is inside the borders
        # Calculate direction vector and travel distance
        direction = velocity / speed
        distance = np.linalg.norm(velocity * delay_left / 1e9)

        # Set mitsuba variant to SCALAR for mobility model
        mi.set_variant('scalar_rgb')

        while True:
            # Create a ray
            ray = mi.Ray3f(mi.Point3f(position), mi.Vector3f(direction), distance, 0.0, [])

            # Calculate the intersection of the ray with the scene
            ray_flags = mi.RayFlags.Minimal
            coherent = False
            active = True
            si = self.mi_scene.ray_intersect(ray, ray_flags, coherent, active)

            if si.is_valid():
                # If the ray hits an object, calculate the reflection

                # The intersection point (position) is set back by one centimeter to prevent cases
                # where the intersection point is found behind a wall
                t = si.t - 0.01
                position = position + t * direction

                # The reflected direction is calculated in the z plane
                n = np.array(si.n)
                n = [n[1], -n[0], 0.0]
                n = n / np.linalg.norm(n)
                direction = - (direction - 2 * np.dot(direction, n) * n)

                velocity = direction * speed
                distance -= t
                delay_left -= (t / speed) * 1e9

                # For testing
                # print(f"CourseChange x = {position[0]}, y = {position[1]}, z = {position[2]}")

            else:
                # If the ray does not hit an object, calculate the next position
                break

        next_position = position + (velocity * delay_left / 1e9)

        self.node_info_dict[node_id]["velocity"] = velocity.tolist()
        self.node_info_dict[node_id]["position"] = next_position.tolist()

        # Set the mitsuba variant back to how Sionna configures it
        if len(gpus) > 0:
            mi.set_variant('cuda_ad_rgb')
        else:
            mi.set_variant('llvm_ad_rgb')


    def remove_all_cached_entries(self, simulation_time):
        for k in list(self.pos_velo_cache.keys()):
            tmp = self.pos_velo_cache[k][:]
            for l in tmp:
                if l.sim_time + l.ttl < simulation_time - self.max_pos_cache_age: # keep last ...
                    self.pos_velo_cache[k].remove(l)


    def get_position_and_velocity(self, node_id, simulation_time):
        # search in cache for pos/velocity
        best_match_value = None
        best_metric = 1e12
        for item in self.pos_velo_cache[node_id]:
            middle = item.sim_time + item.ttl / 2.0
            metric = abs(middle - simulation_time)
            if metric < best_metric and metric <= self.chan_coh_time_mode23:
                best_match_value = item.value
                best_metric = abs(middle - simulation_time)

        if best_match_value is not None:
            # print("Old pos lookupd for node %d at %d: %s" % (node_id, simulation_time, best_match_value[0]))
            return best_match_value

        # sim mobility w/ ray tracing
        value = self.compute_position_and_velocity(node_id, simulation_time)
        # print("New pos computed for node %d at %d: %s" % (node_id, simulation_time, value[0]))

        return value


    def compute_position_and_velocity(self, node_id, simulation_time):
        '''
        Mobility simulation to compute the next position and velocity
        '''

        # If node position is constant, return current position
        if self.node_info_dict[node_id]["model"] == "Constant Position":
            return self.node_info_dict[node_id]["position"], [0.0, 0.0, 0.0]

        # If simulation_time is less than last_update, print a warning and return the last position and velocity
        if simulation_time < self.node_info_dict[node_id]["last update"]:
            warnings.warn(
                f"Trying to calculate the current position and velocity for node {node_id} at time {simulation_time} ns. "
                f"The position and velocity for node {node_id} has already been calculated at time {self.node_info_dict[node_id]['last update']} ns.",
                UserWarning)

            print("Sim time: %d" % simulation_time)
            for item in self.pos_velo_cache[node_id]:
                print(item.sim_time)
            print("====")

            return self.node_info_dict[node_id]["position"], self.node_info_dict[node_id]["velocity"]

        delay_left = self.node_info_dict[node_id]["delay left"]

        # If delay_left equals 0, calculate new walk direction
        if delay_left == 0:
            # Choose speed and direction
            speed = self.get_value(self.node_info_dict[node_id]["speed"])
            direction = round(self.get_value(self.node_info_dict[node_id]["direction"]), 3)

            # Calculate new velocity
            velocity = [np.cos(direction) * speed, np.sin(direction) * speed, 0.0]
            self.node_info_dict[node_id]["velocity"] = velocity

            # Calculate the remaining time to walk in the new direction
            if self.node_info_dict[node_id]["mode"][0] == "Time":
                delay_left = self.node_info_dict[node_id]["mode"][1]
            elif self.node_info_dict[node_id]["mode"][0] == "Distance":
                # If speed is 0, a random walk model with mode "Distance" becomes a constant position model
                if speed == 0:
                    self.node_info_dict[node_id]["model"] = "Constant Position"
                    return self.node_info_dict[node_id]["position"], [0.0, 0.0, 0.0]

                delay_left = abs(self.node_info_dict[node_id]["mode"][1] / speed) * 1e9

            if self.VERBOSE:
                pos = self.node_info_dict[node_id]["position"]
                print(f"CourseChange x = {pos[0]}, y = {pos[1]}, z = {pos[2]}")

        # Walk until the current simulation time is reached
        if self.node_info_dict[node_id]["last update"] + delay_left < simulation_time:
            self.walk(node_id, delay_left)
            self.node_info_dict[node_id]["delay left"] = 0
            return self.compute_position_and_velocity(node_id, simulation_time)
        else:
            self.node_info_dict[node_id]["delay left"] = self.node_info_dict[node_id][
                                                        "last update"] + delay_left - simulation_time
            self.walk(node_id, simulation_time - self.node_info_dict[node_id]["last update"])
            return self.node_info_dict[node_id]["position"], self.node_info_dict[node_id]["velocity"]


    def run(self):
        """
        Handles communication with the ns3 simulator using ZMQ socket
        """
        # Create ZeroMQ socket
        context = zmq.Context()
        socket = zmq.Socket(context, zmq.REP)
        socket.bind("tcp://*:5555")
        socket_open = True
        print("Sionna server socket ready ...")

        last_call_times = []
        num_processed_csi_req = 0
        while socket_open:
            # Receive message from ns3
            from_ns3_message = socket.recv()

            # Deserialize the message
            from_ns3_wrapper = message_pb2.Wrapper()
            from_ns3_wrapper.ParseFromString(from_ns3_message)

            # Prepare the reply message
            to_ns3_wrapper = message_pb2.Wrapper()

            # Fill the reply message
            if from_ns3_wrapper.HasField("sim_init_msg"):
                # handle SimInitMessage & send ACK
                self.store_simulation_info(from_ns3_wrapper.sim_init_msg)
                to_ns3_wrapper.sim_ack.SetInParent()
                print("Sionna server socket connected ...")

            elif from_ns3_wrapper.HasField("channel_state_request"):
                # handle ChannelStateRequest by sending ChannelStateResponse
                start_time = time.time()
                self.calculate_channel_state(from_ns3_wrapper.channel_state_request, to_ns3_wrapper)
                call_time = time.time() - start_time
                last_call_times.append(call_time)
                num_processed_csi_req += 1

                if self.VERBOSE or num_processed_csi_req % 1 == 0:
                    print("t=%.9fs: average event processing time: %.2f sec"
                          % (from_ns3_wrapper.channel_state_request.time/1e9, np.nanmean(last_call_times)))

            elif from_ns3_wrapper.HasField("sim_close_request"):
                socket_open = False
                to_ns3_wrapper.sim_ack.SetInParent()

            # Serialize and send the reply message
            socket.send(to_ns3_wrapper.SerializeToString())

        socket.close()
        print("Mode: %d , submode: %d , NoCSI: %d , avgevent: %.2f" % (self.mode, self.sub_mode, num_processed_csi_req, np.nanmean(last_call_times)))
        print("Sionna server socket closed.")
        # cleanup sionna


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--single_run", help="Whether not to terminate after single run", action='store_true')
    parser.add_argument("--rt_calc_diffraction", help="Calc diffraction in raytracing", action='store_true')
    parser.add_argument("--rt_max_depth", type=int, default=6, help="Calc diffraction in raytracing")
    parser.add_argument("--rt_max_parallel_links", type=int, default=4, help="Max no. of receivers")
    parser.add_argument("--est_csi", help="Whether to estimate complex CSI per OFDM subcarrier", action='store_true')
    parser.add_argument("--verbose", help="Whether to run in verbose mode", action='store_true')
    args = parser.parse_args()

    print("ns3sionna v0.3")
    while True:
        print("Using config: rt_calc_diffraction=%s, rt_max_depth=%s, rt_max_parallel_links=%d, est_csi=%r" % (args.rt_calc_diffraction, args.rt_max_depth, args.rt_max_parallel_links, args.est_csi))
        print("Waiting for new job ...")
        env = SionnaEnv(args.rt_calc_diffraction, args.rt_max_depth, args.rt_max_parallel_links, args.est_csi, VERBOSE=args.verbose)
        env.run()

        if args.single_run:
            break
