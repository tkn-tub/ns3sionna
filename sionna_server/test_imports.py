try:
    import zmq
    print("Package zmq is installed")
except ImportError:
    print("Package zmq is not installed")

try:
    import message_pb2
    print("Package message_pb2 is installed")
except ImportError:
    print("Package message_pb2 is not installed")

try:
    import cProfile
    print("Package cProfile is installed")
except ImportError:
    print("Package cProfile is not installed")

try:
    import pstats
    print("Package pstats is installed")
except ImportError:
    print("Package pstats is not installed")

try:
    import os
    print("Package os is installed")
except ImportError:
    print("Package os is not installed")

try:
    import sionna
    print("Package sionna is installed")
    print(sionna.__version__)

except ImportError:
    print("Package sionna is not installed")

try:
    import tensorflow as tf
    print("Package tensorflow is installed")

    gpu_num = 0 # Use "" to use the CPU
    os.environ["CUDA_VISIBLE_DEVICES"] = f"{gpu_num}"
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

    gpus = tf.config.list_physical_devices('GPU')
    if gpus:
        try:
            tf.config.experimental.set_memory_growth(gpus[0], True)
        except RuntimeError as e:
            print(e)
    tf.get_logger().setLevel('ERROR')

    print("gpus_num:", gpu_num)
    print("gpus:", gpus)

except ImportError:
    print("Package tensorflow is not installed")

try:
    import mitsuba as mi
    print("Package mitsuba is installed")
except ImportError:
    print("Package mitsuba is not installed")

try:
    import numpy as np
    print("Package numpy is installed")
except ImportError:
    print("Package numpy is not installed")

try:
    import warnings
    print("Package warnings is installed")
except ImportError:
    print("Package warnings is not installed")

try:
    import time
    print("Package time is installed")
except ImportError:
    print("Package time is not installed")

try:
    import re
    print("Package re is installed")
except ImportError:
    print("Package re is not installed")

try:
    import matplotlib.pyplot as plt
    print("Package matplotlib is installed")
except ImportError:
    print("Package matplotlib is not installed")
