#!/usr/bin/bash

# MEU PATH - PROTOTIPO (AUTOMATIZAR ESSA TAREFA)
PATH_SLICER='Desktop/JoseCarlos/3d_slicer/Slicer-5.0.2-linux-amd64'

exec ~/$PATH_SLICER/Slicer --no-main-window --python-script registration/script.py
