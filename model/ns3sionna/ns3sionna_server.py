import sionna.rt # must be the first import as otherwise Python crashes

import os
import argparse
import numpy as np
import time
import GPUtil
import zmq
import gc

from common import message_pb2
from common.message_debug import *

from collections import deque
import warnings

import tensorflow as tf
import mitsuba as mi
import math

from ns3sionna_utils import subcarrier_frequencies, compute_coherence_time, SECOND, MILLISECOND, coherence_from_velocities, \
    MAX_COHERENCE_TIME

import sionna
from sionna.rt import load_scene, Camera, Transmitter, Receiver, PlanarArray, PathSolver

# import mobility models
from mobility import *

class SionnaEnv:

    # just compute the given single point-to-point channel
    MODE_P2P        = 1
    # extend the given P2P channel to include all receivers to take broadcast nature of wireless into account
    MODE_P2MP       = 2
    # compute also future channels; used only if the transmitter is static (receivers are mobile)
    MODE_P2MP_LAH   = 3

    """
    This class represents the Sionna component of ns3sionna. It represents the environment where the node
    placement, mobility is controlled from the client component of ns3sionna. For IPC ZMQ is used.

    author: Pilz, Zubow
    """
    def __init__(self, model_folder='./models/', rt_fast=False, default_mode=MODE_P2P, rt_max_parallel_links=256, est_csi=True, VERBOSE=True,
                 CHECKS_ENABLED=True):
        self.model_folder = model_folder
        self.rt_fast = rt_fast
        if rt_fast:
            self.rt_max_depth = 3  # very small
            self.rt_samples_per_src = 10 ** 6
            self.rt_los = True  # compute and include the direct Line-of-Sight path when it exists
            self.rt_specular_reflection = True  # Can rays bounce off surfaces?
            self.rt_diffuse_reflection = False
            self.rt_refraction = True  # Can rays pass through materials?
            self.rt_synthetic_array = False  # Set True for fast simulation using one ray trace for whole array; per-element effects computed analytically
            self.rt_diffraction = False  # costly
            self.rt_edge_diffraction = False  # rays that bend around edges
            self.rt_diffraction_lit_region = False  # higher physical accuracy; for mmWave or THz channels
        else: # realistic but slow
            self.rt_max_depth = 5  # sufficient even for rich multipath
            self.rt_samples_per_src = 10 ** 6  # 10 ** 6
            self.rt_los = True  # compute and include the direct Line-of-Sight path when it exists
            self.rt_specular_reflection = True  # Can rays bounce off surfaces?
            self.rt_diffuse_reflection = True
            self.rt_refraction = True  # Can rays pass through materials?
            self.rt_synthetic_array = False  # Set True for fast simulation using one ray trace for whole array; per-element effects computed analytically
            self.rt_diffraction = True  # costly
            self.rt_edge_diffraction = True  # rays that bend around edges
            self.rt_diffraction_lit_region = True  # higher physical accuracy; for mmWave or THz channels

        # default mode
        self.default_mode = default_mode

        # maximum number of parallel computations (needed to fit GPU)
        self.rt_max_parallel_links = rt_max_parallel_links

        # estimate small-scale fading
        self.est_csi = est_csi

        print(f'Init ns3sionna with rt_fast={rt_fast}, est_csi={est_csi}')

        self.VERBOSE = VERBOSE
        self.CHECKS_ENABLED = CHECKS_ENABLED

        # check GPU support
        self.gpus = tf.config.list_physical_devices("GPU")

        if len(self.gpus) > 0:
            print("GPU support detected; no GPUs:", self.gpus)
        else:
            print("Using CPU backend")

        self.disp_r = 10
        # storing information about every node under simulation
        self.node_info = {}
        # all node which are currently placed on the scene
        self.placed_radio_node_names = []


    def init_simulation_env(self, sim_init_msg):
        '''
        Initializes the Sionna environment
        :param sim_init_msg: the received ZMQ message
        :return: (success, error_msg)
        '''
        if self.VERBOSE:
            print_sim_init(sim_init_msg)

        # Load the sionna scene
        filepath = os.path.join(self.model_folder, sim_init_msg.scene_fname)
        try:
            self.scene = load_scene(filepath)
        except Exception as e:
            return False, "Failed to load scene file in: " + filepath + ", error: " + str(e)

        self.bbox = self.scene.mi_scene.bbox()

        if self.VERBOSE:
            # show some stats about the scene
            dx = self.bbox.max.x - self.bbox.min.x
            dy = self.bbox.max.y - self.bbox.min.y
            dz = self.bbox.max.z - self.bbox.min.z
            print(f'Scenario with dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}')

        # set mode/submode if valid
        if sim_init_msg.mode > -1:
            self.mode = sim_init_msg.mode
        else:
            self.mode = self.default_mode # default mode

        # todo: use submode
        if sim_init_msg.sub_mode > -1:
            self.sub_mode = sim_init_msg.sub_mode
        else:
            self.sub_mode = self.rt_max_parallel_links

        self.time_evo_model = sim_init_msg.time_evo_model

        # Set scene parameters
        self.scene.frequency = sim_init_msg.frequency * 1e6
        self.scene.bandwidth = sim_init_msg.channel_bw * 1e6 # max channel bandwidth

        self.fc = sim_init_msg.frequency * 1e6
        self.fft_size = sim_init_msg.fft_size  # max FFT size
        # todo: min Tc is not used
        self.min_coherence_time_ms = sim_init_msg.min_coherence_time_ms # min Tc
        self.subcarrier_spacing = sim_init_msg.subcarrier_spacing # in Hz

        print(f'Operating in mode: {self.mode}, sub_mode: {self.sub_mode}, time_evo_model: {self.time_evo_model}'
              f', fc: {sim_init_msg.frequency} MHz, B: {sim_init_msg.channel_bw} MHz, FFT size: {self.fft_size}')

        # Subcarrier frequencies
        self.frequencies = subcarrier_frequencies(num_subcarriers=self.fft_size, subcarrier_spacing=self.subcarrier_spacing)

        # Set the random seed for reproducibility
        np.random.seed(sim_init_msg.seed)
        tf.random.set_seed(sim_init_msg.seed)
        self.my_seed = sim_init_msg.seed

        # configure mobility models
        self._init_mobility(sim_init_msg)

        # for mode 3 if only constant speed model supported
        if self.mode == SionnaEnv.MODE_P2MP_LAH:
            speed_arr = []
            for node_id in list(self.node_info.keys()):
                if isinstance(self.node_info[node_id], RandomWalkMobility):
                    if self.node_info[node_id].speed != RandomWalkMobility.SPEED_CONSTANT:
                        warnings.warn(f"Only constant speed model is supported when using mode P2MP(LAH); switching to mode P2P.", UserWarning)
                        self.mode = SionnaEnv.MODE_P2MP
                        break
                    else:
                        speed_arr.append(self.node_info[node_id].speed_params[0])
            # compute coherence time assuming worst case: fastest nodes move away from each other
            speed_arr.sort(reverse=True)
            if len(speed_arr) >= 2:
                self.chan_coh_time_mode3 = compute_coherence_time(speed_arr[0] + speed_arr[1], self.fc, model='rappaport2')
            elif len(speed_arr) == 1:
                self.chan_coh_time_mode3 = compute_coherence_time(speed_arr[0], self.fc, model='rappaport2')
            else:
                self.chan_coh_time_mode3 = MAX_COHERENCE_TIME

            print(f'Running mode=3 w/ Tc: {self.chan_coh_time_mode3/1e6}ms')

        # Configure antenna array for all transmitters/receivers
        self.scene.tx_array = PlanarArray(num_rows=1, num_cols=1, vertical_spacing=0.5, horizontal_spacing=0.5,
                                     pattern="tr38901", polarization="V")

        self.scene.rx_array = PlanarArray(num_rows=1, num_cols=1, vertical_spacing=0.5, horizontal_spacing=0.5,
                                     pattern="dipole", polarization="V")

        # set current sim time to 0ns
        self.sim_time = 0

        return True, "OK"


    def compute_cfr(self, csi_req, reply_wrapper):

        tx_node_id = csi_req.tx_node
        rx_node_id = csi_req.rx_node

        if self.mode == SionnaEnv.MODE_P2MP_LAH:
            if isinstance(self.node_info[tx_node_id], RandomWalkMobility) and isinstance(self.node_info[rx_node_id], ConstantMobility):
                # Exploit channel reciprocity - swap mobile TX with static RX
                csi_req.tx_node = rx_node_id
                csi_req.rx_node = tx_node_id

        # check if mode 3 can be used
        if self.mode == SionnaEnv.MODE_P2MP_LAH and isinstance(self.node_info[tx_node_id], ConstantMobility):
            # mode=3 is feasible if TX is fixed
            self.compute_cfr_with_lookahead(csi_req, reply_wrapper)
        else:
            req_mode = self.mode
            # mode=1/2 or if TX node is mobile
            if self.mode == SionnaEnv.MODE_P2MP_LAH:
                req_mode = SionnaEnv.MODE_P2MP
                print(f'Fallback to mode={req_mode} as TX node is mobile')

            self.compute_cfr_classic(csi_req, reply_wrapper, req_mode)


    def compute_cfr_with_lookahead(self, csi_req, reply_wrapper):
        '''
        Compute the requested CFR
        :param csi_req: received CSI request (ZMQ)
        :param reply_wrapper: the response
        '''

        if self.VERBOSE:
            print_csi_request(csi_req)

        tx_node_id = csi_req.tx_node
        rx_node_id = csi_req.rx_node # this rx node must be included in result set
        req_sim_time = csi_req.time # we need CFR at that point in time [ns]

        assert self.time_evo_model == 'position'

        # execute mobility

        # update position of all nodes
        nodes_to_update = list(self.node_info.keys())

        # compute look-ahead
        look_ahead = math.floor(self.rt_max_parallel_links / (len(nodes_to_update) - 1))

        print(f'compute CFR to #RX={len(nodes_to_update) - 1} with LAH={look_ahead}')

        # sim future node positions
        lah_time_vec = []
        for lah_i in range(look_ahead):
            # move in time
            dt = req_sim_time - self.sim_time

            csi_tc_arr = []
            for node_id in nodes_to_update:
                # perform walk
                self._walk(node_id, dt)
                if node_id != tx_node_id:
                    tc = coherence_from_velocities(self.node_info[node_id].velocity,
                                                    self.node_info[tx_node_id].velocity, self.fc,
                                                    pos_tx=self.node_info[node_id].pos,
                                                    pos_rx=self.node_info[tx_node_id].pos)
                    csi_tc_arr.append(tc)

            # take the worst case Tc from all RX nodes
            Tc_p2mp = int(np.min(np.asarray(csi_tc_arr)))

            # update time
            self.sim_time = req_sim_time
            lah_time_vec.append(req_sim_time)
            # new req time is old + Tc
            req_sim_time = req_sim_time + Tc_p2mp


        # place TX and RX nodes together with their future positions
        rx_nodes = nodes_to_update
        rx_nodes.remove(tx_node_id)

        self._place_tx_rx_nodes_with_lah(lah_time_vec, tx_node_id, rx_nodes)

        # create pathsolver; todo: check reuse
        p_solver  = PathSolver()

        # Compute propagation paths
        paths = p_solver(scene=self.scene,
                         max_depth=self.rt_max_depth,
                         samples_per_src=self.rt_samples_per_src,
                         los=self.rt_los,
                         specular_reflection=self.rt_specular_reflection,  # Can rays bounce off surfaces?
                         diffuse_reflection=self.rt_diffuse_reflection,
                         refraction=self.rt_refraction,  # Can rays pass through materials?
                         synthetic_array=self.rt_synthetic_array,
                         diffraction=self.rt_diffraction,  # costly
                         edge_diffraction=self.rt_edge_diffraction,  # rays that bend around edges
                         diffraction_lit_region=self.rt_diffraction_lit_region)  # higher physical accuracy

        # AZU: sampling_frequency is only used if num_time_steps > 1
        # a: shape [num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths, num_time_steps],
        a, tau = paths.cir(sampling_frequency=1e9, normalize_delays=False, out_type="numpy")

        # shape: [num_rx, num_rx_ant, num_tx, num_tx_ant, num_ofdm_symbols, num_subcarriers]
        h_raw = paths.cfr(frequencies=self.frequencies,
                  sampling_frequency=1.0,  # not used
                  num_time_steps=1,
                  normalize_delays=True,
                  # If set to True, path delays are normalized such that the first path between any pair of
                  # antennas of a transmitter and receiver arrives at tau=0
                  normalize=False,  # Normalize energy
                  out_type="numpy")

        # max single transmitter
        assert h_raw.shape[2] == 1
        num_computed_lnks = h_raw.shape[0]

        # Create ZMQ response
        chan_response = reply_wrapper.channel_state_response

        Tc_p2mp_lah = []
        for lah_time_idx, lah_time in enumerate(lah_time_vec): # iterate over time
            csi = chan_response.csi.add()

            csi.start_time = lah_time
            # tx node is fixed
            tx_pos = self.node_info[tx_node_id].pos
            csi.tx_node.id = tx_node_id
            csi.tx_node.position.x = tx_pos[0]
            csi.tx_node.position.y = tx_pos[1]
            csi.tx_node.position.z = tx_pos[2]

            # get receiver(s)
            csi_tc_arr = []
            for curr_rx_id, curr_rx_node in enumerate(rx_nodes):

                rx_id = lah_time_idx * len(rx_nodes) + curr_rx_id

                lnk_tau = np.squeeze(tau[rx_id, :, :, :, :])
                lnk_delay = int(round(np.min(lnk_tau[lnk_tau >= 0] * 1e9), 0))

                h = np.squeeze(h_raw[rx_id, :, :, :, :, ])

                # see Parseval's theorem
                lnk_loss = float(-10 * np.log10(np.mean(np.abs(h) ** 2)))

                # for frequency-selective channel
                power = np.mean(np.abs(h) ** 2)  # shape [batch_size, 1, 1, 1]

                h_normalized = h / np.sqrt(power)

                # plausibility test
                if self.CHECKS_ENABLED:
                    power_normalized = np.mean(np.abs(h_normalized) ** 2)
                    assert math.isclose(power_normalized, 1.0, rel_tol=1e-3)   # Should be close to 1

                if self.VERBOSE:
                    print(f'{lah_time/1e9}s: {tx_node_id}->{curr_rx_node} lnk_delay = {lnk_delay}ns, wb_loss = {lnk_loss:.3f}dB, CFR shape: {h_normalized.shape}')

                rx_node_info = csi.rx_nodes.add()
                rx_pos = self.node_info[curr_rx_node].get_pos_at(lah_time)
                rx_node_info.id = curr_rx_node
                rx_node_info.position.x = rx_pos[0]
                rx_node_info.position.y = rx_pos[1]
                rx_node_info.position.z = rx_pos[2]
                rx_node_info.delay = lnk_delay
                rx_node_info.wb_loss = lnk_loss

                if self.est_csi:
                    rx_node_info.frequencies.extend(self.frequencies.tolist())
                    rx_node_info.csi_imag.extend(np.imag(h_normalized).tolist())
                    rx_node_info.csi_real.extend(np.real(h_normalized).tolist())

                tc = coherence_from_velocities(self.node_info[curr_rx_node].get_velo_at(lah_time),
                                                self.node_info[tx_node_id].velocity, self.fc,
                                                pos_tx=self.node_info[curr_rx_node].get_pos_at(lah_time),
                                                pos_rx=self.node_info[tx_node_id].pos)

                rx_node_info.end_time2 = csi.start_time + tc
                csi_tc_arr.append(tc)

            # take the worst case Tc from all RX nodes
            Tc_p2mp = int(np.min(np.asarray(csi_tc_arr)))
            Tc_p2mp_lah.append(Tc_p2mp)
            csi.end_time = csi.start_time + Tc_p2mp - 1 # -1ns to have non-overlapping intervals

        print(f'{self.sim_time / 1e9}s: Computed CSI with Tc: {round(np.asarray(Tc_p2mp_lah) / 1e6,2)}ms, #links: {num_computed_lnks}')


    def _place_tx_rx_nodes_with_lah(self, lah_time_vec: list, tx_node: int, rx_nodes: list):
        '''
        Place the given nodes together with their lookahead positions in the scenario
        :param lah_time_vec: time vector containing the lah time vector
        :param tx_node: the transmitting node
        :param rx_nodes: the receiver nodes
        '''

        # remove old tx and rx nodes
        for placed_node in self.placed_radio_node_names:
            self.scene.remove(placed_node)
        self.placed_radio_node_names.clear()

        # only supported if TX is fixed
        fixed_tx_node = isinstance(self.node_info[tx_node], ConstantMobility)
        assert fixed_tx_node

        # Create transmitter
        tx_pos = self.node_info[tx_node].pos
        tx_node_name = "tx"
        tx = Transmitter(name=tx_node_name, position=tx_pos, orientation=[0, -180, 0], display_radius=self.disp_r)
        self.scene.add(tx)
        self.placed_radio_node_names.append(tx_node_name)

        for lah_time_idx, lah_time in enumerate(lah_time_vec):
            # Create a receiver(s)
            for rx_node_i in rx_nodes:
                rx_node_name = "rx" + str(rx_node_i) + "." + str(lah_time_idx)
                rx_pos = self.node_info[rx_node_i].get_pos_at(lah_time)
                rx = Receiver(name=rx_node_name, position=rx_pos, orientation=[0, -180, 0], display_radius=self.disp_r)
                self.scene.add(rx)
                self.placed_radio_node_names.append(rx_node_name)


    def compute_cfr_classic(self, csi_req, reply_wrapper, req_mode):
        '''
        Compute the requested CFR
        :param csi_req: received CSI request (ZMQ)
        :param reply_wrapper: the response
        '''

        if self.VERBOSE:
            print_csi_request(csi_req)

        tx_node_id = csi_req.tx_node
        rx_node_id = csi_req.rx_node # this rx node must be included in result set
        req_sim_time = csi_req.time # we need CFR at that point in time [ns]

        if self.time_evo_model == 'doppler':
            #(lnk_delay, lnk_loss, h_normalized) = self.compute_cfr_via_doppler()
            pass
        else: # position=based
            (rx_nodes, lnk_delay, lnk_loss, h_normalized) = self._compute_cfr_via_position(req_sim_time, tx_node_id, rx_node_id, req_mode)

        # Create ZMQ response
        chan_response = reply_wrapper.channel_state_response
        csi = chan_response.csi.add()

        csi.start_time = self.sim_time
        # tx node info
        tx_pos = self.node_info[tx_node_id].pos
        csi.tx_node.id = tx_node_id
        csi.tx_node.position.x = tx_pos[0]
        csi.tx_node.position.y = tx_pos[1]
        csi.tx_node.position.z = tx_pos[2]

        csi_tc_arr = []
        # for all rx nodes
        for idx, comp_rx_node_id in enumerate(rx_nodes):
            rx_node_info = csi.rx_nodes.add()
            rx_pos = self.node_info[comp_rx_node_id].pos
            rx_node_info.id = comp_rx_node_id
            rx_node_info.position.x = rx_pos[0]
            rx_node_info.position.y = rx_pos[1]
            rx_node_info.position.z = rx_pos[2]
            rx_node_info.delay = lnk_delay[idx]
            rx_node_info.wb_loss = lnk_loss[idx]

            if self.est_csi:
                rx_node_info.frequencies.extend(self.frequencies.tolist())
                rx_node_info.csi_imag.extend(np.imag(h_normalized[idx]).tolist())
                rx_node_info.csi_real.extend(np.real(h_normalized[idx]).tolist())

            # compute coherence time: with direction vectors you can compute the radial (projected) relative
            # speed directly and from that the Doppler and coherence time.
            tc = coherence_from_velocities(self.node_info[comp_rx_node_id].velocity,
                                            self.node_info[tx_node_id].velocity, self.fc,
                                            pos_tx=self.node_info[comp_rx_node_id].pos,
                                            pos_rx=self.node_info[tx_node_id].pos)
            rx_node_info.end_time2 = tc
            csi_tc_arr.append(tc)

        # take the worst case Tc from all RX nodes
        Tc_p2mp = int(np.min(np.asarray(csi_tc_arr)))

        print(f'{self.sim_time / 1e9}s: Computed CSI with Tc: {round(Tc_p2mp / 1e6,2)}ms, #links: {len(rx_nodes)}')

        csi.end_time = self.sim_time + Tc_p2mp


    def _walk(self, node_id, dt):
        """
        Move the given node to the given time interval.
        :param node_id: node_id of the node to move
        :param dt: time interval in ns
        """
        init_dt = dt

        pos = self.node_info[node_id].pos
        velocity = self.node_info[node_id].velocity

        speed = np.linalg.norm(velocity)
        if speed == 0:
            return

        # Check if the next position is inside the borders
        # Calculate direction vector and travel distance
        direction = velocity / speed
        distance = np.linalg.norm(velocity * dt / 1e9) # convert dt into sec

        while True:
            # Create a ray
            ray = mi.Ray3f(mi.Point3f(pos), mi.Vector3f(direction))
            ray.maxt = mi.Float(distance)

            # Calculate the intersection of the ray with the scene
            si = self.scene.mi_scene.ray_intersect(ray, mi.RayFlags.Minimal, False, True)

            if si.is_valid():
                # If the ray hits an object, calculate the reflection

                # The intersection point (position) is set back by one centimeter to prevent cases
                # where the intersection point is found behind a wall
                t_np = si.t.numpy()
                t = float(t_np[0]) - 0.01
                pos = pos + t * direction

                # The reflected direction is calculated in the z plane
                n = np.squeeze(si.n.numpy())
                n = [n[1], -n[0], 0.0]
                n = n / np.linalg.norm(n)

                mob_theta = self.node_info[node_id].get_next_direction_angle()
                direction = - (direction - 2 * (np.dot(direction, n) + mob_theta) * n)

                velocity = direction * speed

                # make sure we do not change speed
                velocity = (velocity / np.linalg.norm(velocity)) * speed

                distance -= t
                dt -= (t / speed) * 1e9

            else:
                # If the ray does not hit an object, calculate the next position
                break

        next_pos = pos + (velocity * dt / 1e9)

        # update node pos & velocity
        self.node_info[node_id].update_pos(self.sim_time + init_dt, next_pos, velocity, False)
        # check if new velocity must be set
        self.node_info[node_id].check_set_new_velocity(self.sim_time + init_dt, distance)


    def _place_tx_rx_node(self, tx_node: int, rx_nodes: list):
        '''
        Place the given nodes in the scenario
        :param tx_node: the transmitting node
        :param rx_nodes: the receiver nodes
        '''

        # remove old tx and rx nodes
        for placed_node in self.placed_radio_node_names:
            self.scene.remove(placed_node)
        self.placed_radio_node_names.clear()

        # Create transmitter
        tx_pos = self.node_info[tx_node].pos
        tx_node_name = "tx"
        tx = Transmitter(name=tx_node_name, position=tx_pos, orientation=[0, -180, 0], display_radius=self.disp_r)
        self.scene.add(tx)
        self.placed_radio_node_names.append(tx_node_name)

        # Create a receiver(s)
        for rx_node_i in rx_nodes:
            rx_node_name = "rx" + str(rx_node_i)
            rx_pos = self.node_info[rx_node_i].pos
            rx = Receiver(name=rx_node_name, position=rx_pos, orientation=[0, -180, 0], display_radius=self.disp_r)
            self.scene.add(rx)
            self.placed_radio_node_names.append(rx_node_name)


    def _compute_cfr_via_position(self, req_sim_time, tx_node, rx_node, req_mode):
        '''
        Compute the link propagation delay, wideband loss and normalized CFR
        :param req_sim_time: current simulation time
        :param tx_node: the transmitter node id
        :param rx_node: the receiver node id
        :return: (list(rx_node), list(link propagation delay), list(wideband loss), list(normalized CFR))
        '''

        # execute mobility
        dt = req_sim_time - self.sim_time

        # estimate the node we need to update their position
        if req_mode == SionnaEnv.MODE_P2P:
            nodes_to_update = [tx_node, rx_node] # only TX and RX
        else:
            # both P2MP and P2MP_LAH
            nodes_to_update = list(self.node_info.keys())

        for node_id in nodes_to_update:
            self._walk(node_id, dt)

        # update time
        self.sim_time = req_sim_time

        # place TX and RX
        rx_nodes = nodes_to_update
        rx_nodes.remove(tx_node)
        self._place_tx_rx_node(tx_node, rx_nodes)

        # create pathsolver; todo: check reuse
        p_solver  = PathSolver()

        # Compute propagation paths
        paths = p_solver(scene=self.scene,
                         max_depth=self.rt_max_depth,
                         samples_per_src=self.rt_samples_per_src,
                         los=self.rt_los,
                         specular_reflection=self.rt_specular_reflection,  # Can rays bounce off surfaces?
                         diffuse_reflection=self.rt_diffuse_reflection,
                         refraction=self.rt_refraction,  # Can rays pass through materials?
                         synthetic_array=self.rt_synthetic_array,
                         diffraction=self.rt_diffraction,  # costly
                         edge_diffraction=self.rt_edge_diffraction,  # rays that bend around edges
                         diffraction_lit_region=self.rt_diffraction_lit_region)  # higher physical accuracy

        # AZU: sampling_frequency is only used if num_time_steps > 1
        # a: shape [num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths, num_time_steps],
        a, tau = paths.cir(sampling_frequency=1e9, normalize_delays=False, out_type="numpy")

        # shape: [num_rx, num_rx_ant, num_tx, num_tx_ant, num_ofdm_symbols, num_subcarriers]
        h_raw = paths.cfr(frequencies=self.frequencies,
                  sampling_frequency=1.0,  # not used
                  num_time_steps=1,
                  normalize_delays=True,
                  # If set to True, path delays are normalized such that the first path between any pair of
                  # antennas of a transmitter and receiver arrives at tau=0
                  normalize=False,  # Normalize energy
                  out_type="numpy")

        lnk_delay_arr = []
        lnk_loss_arr = []
        h_normalized_arr = []

        for rx_id, curr_rx_node in enumerate(rx_nodes):
            lnk_tau = np.squeeze(tau[rx_id, :, :, :, :])
            lnk_delay = int(round(np.min(lnk_tau[lnk_tau >= 0] * 1e9), 0))

            h = np.squeeze(h_raw[rx_id, :, :, :, :, ])

            # see Parseval's theorem
            lnk_loss = float(-10 * np.log10(np.mean(np.abs(h) ** 2)))

            # for frequency-selective channel
            power = np.mean(np.abs(h) ** 2)  # shape [batch_size, 1, 1, 1]

            h_normalized = h / np.sqrt(power)

            # plausibility test
            if self.CHECKS_ENABLED:
                power_normalized = np.mean(np.abs(h_normalized) ** 2)
                assert math.isclose(power_normalized, 1.0, rel_tol=1e-3)   # Should be close to 1

            if self.VERBOSE:
                print(f'{self.sim_time/1e9}s: lnk_delay = {lnk_delay}ns, wb_loss = {lnk_loss:.3f}dB, CFR shape: {h_normalized.shape}')

            lnk_delay_arr.append(lnk_delay)
            lnk_loss_arr.append(lnk_loss)
            h_normalized_arr.append(h_normalized)

        return rx_nodes, lnk_delay_arr, lnk_loss_arr, h_normalized_arr


    def _get_mobility_history(self, node_id):
        ts = sorted(self.node_info[node_id].pos_history.keys())
        pos = [self.node_info[node_id].pos_history[t] for t in ts]
        return ts, pos


    def _init_mobility(self, sim_init_msg):
        # Store information about each node: ID, mobility model
        for node_info in sim_init_msg.nodes:
            if (node_info.HasField("constant_position_model")):
                # fixed position; no mobility
                pos = node_info.constant_position_model.position
                self.node_info[node_info.id] = ConstantMobility(node_info.id, [pos.x, pos.y, pos.z])
            elif (node_info.HasField("random_walk_model")):
                # mobile scenario
                random_walk_model = node_info.random_walk_model
                pos = random_walk_model.position

                mode = None
                if random_walk_model.HasField("wall_value"):
                    mode = RandomWalkMobility.MODE_WALL
                    mode_params = random_walk_model.wall_value
                elif random_walk_model.HasField("time_value"):
                    mode = RandomWalkMobility.MODE_TIME
                    mode_params = random_walk_model.time_value
                elif random_walk_model.HasField("distance_value"):
                    mode = RandomWalkMobility.MODE_DISTANCE
                    mode_params = random_walk_model.distance_value

                speed = None
                if random_walk_model.speed.HasField("uniform"):
                    speed = RandomWalkMobility.SPEED_UNIFORM
                    speed_params = (random_walk_model.speed.uniform.min, random_walk_model.speed.uniform.max)
                elif random_walk_model.speed.HasField("constant"):
                    speed = RandomWalkMobility.SPEED_CONSTANT
                    speed_params = (random_walk_model.speed.constant.value,)
                elif random_walk_model.speed.HasField("normal"):
                    speed = RandomWalkMobility.SPEED_NORMAL
                    speed_params = (random_walk_model.speed.normal.mean, random_walk_model.speed.normal.variance)

                direction = None
                if random_walk_model.direction.HasField("uniform"):
                    direction = RandomWalkMobility.DIRECTION_UNIFORM
                    direction_params = (random_walk_model.direction.uniform.min,
                                        random_walk_model.direction.uniform.max)
                elif random_walk_model.direction.HasField("constant"):
                    direction = RandomWalkMobility.DIRECTION_CONSTANT
                    direction_params = (random_walk_model.direction.constant.value,)
                elif random_walk_model.direction.HasField("normal"):
                    direction = RandomWalkMobility.DIRECTION_NORMAL
                    direction_params = (random_walk_model.direction.normal.mean,
                                        random_walk_model.direction.normal.variance)

                self.node_info[node_info.id] = RandomWalkMobility(node_info.id, [pos.x, pos.y, pos.z],
                                                                  mode, mode_params, speed, speed_params,
                                                                  direction, direction_params)


    def run(self):
        '''
        Handles communication with the ns3 simulator using ZMQ socket
        '''

        context = zmq.Context()
        socket = zmq.Socket(context, zmq.REP)
        socket.bind("tcp://*:5555")

        print("Sionna server socket ready ...")

        last_call_times = deque(maxlen=10)
        num_csi_req = 0

        do_terminate = False
        while not do_terminate:
            # Receive message from ns3 simulator
            ns3_msg_str = socket.recv()

            # Deserialize received message
            ns3_msg = message_pb2.Wrapper()
            ns3_msg.ParseFromString(ns3_msg_str)

            # Prepare reply message
            resp_msg = message_pb2.Wrapper()

            # Fill the reply message
            if ns3_msg.HasField("sim_init_msg"):
                # handle SimInitMessage & send ACK
                successful, error_msg = self.init_simulation_env(ns3_msg.sim_init_msg)
                resp_msg.sim_ack.no_error = successful
                resp_msg.sim_ack.error_msg = error_msg
                resp_msg.sim_ack.SetInParent()

                if successful:
                    print("Sionna server init sucessful ...")
                else:
                    print("Sionna server init failed ...")
                    do_terminate = True

            elif ns3_msg.HasField("channel_state_request"):
                # handle ChannelStateRequest by sending ChannelStateResponse
                start_time = time.time()
                self.compute_cfr(ns3_msg.channel_state_request, resp_msg)
                last_call_times.append(time.time() - start_time)

                if self.VERBOSE:
                    avg_call_time = sum(last_call_times) / len(last_call_times)
                    print("t=%.9fs: average event processing time: %.2f sec"
                          % (ns3_msg.channel_state_request.time/1e9, avg_call_time))
                    # show GPU load
                    if len(self.gpus) > 0:
                        GPUtil.showUtilization()

            elif ns3_msg.HasField("sim_close_request"):
                do_terminate = True
                resp_msg.sim_ack.SetInParent()

            # Serialize and send the reply message
            socket.send(resp_msg.SerializeToString())

        socket.close()
        print("Handled no. CSI req: %d" % num_csi_req)
        print("Sionna server socket closed.")


    def release(self):
        # delete / release the scene before loading a new one
        del self.scene
        gc.collect()  # force garbage collection


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_folder", type=str, default='models/', help="The folder containing the XML files of the scenes")
    parser.add_argument("--single_run", help="Whether not to terminate after single run", action='store_true')
    parser.add_argument("--default_mode", type=int, default=SionnaEnv.MODE_P2MP, help="Which mode to use if not set by ns3")
    parser.add_argument("--rt_fast", help="Use simplified raytracing for faster computations", action='store_true')
    parser.add_argument("--rt_max_parallel_links", type=int, default=256, help="Max no. of link simulated at once; depends on GPU memory")
    parser.add_argument("--est_csi", help="Whether to estimate complex CSI per OFDM subcarrier", type=bool, default=True)
    parser.add_argument("--verbose", help="Whether to run in verbose mode", action='store_true')
    args = parser.parse_args()

    print("ns3sionna v1.0")
    while True:
        print("Using config: model_folder=%s, single_run=%s, mode=%d, rt_fast=%s, rt_max_parallel_links=%d, est_csi=%r"
              % (args.model_folder, args.single_run, args.default_mode, args.rt_fast, args.rt_max_parallel_links, args.est_csi))
        print("Waiting for new job ...")
        env = SionnaEnv(args.model_folder, args.rt_fast, args.default_mode, args.rt_max_parallel_links,
                        args.est_csi, VERBOSE=args.verbose)
        env.run()

        if args.single_run:
            break

