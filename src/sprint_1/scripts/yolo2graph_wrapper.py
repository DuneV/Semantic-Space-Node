#!/home/krita/mujoco_venv/mujoco-env/bin/python3

import sys
import os

venv_path = "/home/krita/mujoco-env"
sys.path.insert(0, os.path.join(venv_path, "lib", "python3.12", "site-packages"))

from sprint_1.yolo2graph import main

if __name__ == '__main__':
    main()