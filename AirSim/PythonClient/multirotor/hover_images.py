# goal of this script is to hover the quadrotor at a certain altitude and capture RGB, depth, and semantic segmentation images
import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

