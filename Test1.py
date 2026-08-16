import sys
import os

# Update this path to match your actual directory structure
EGG_PATH = r"C:\Users\prana\Desktop\TiHAN\CARLA_CODESYS_simulation\CARLA_0.9.11\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.11-py3.7-win-amd64.egg"

if os.path.exists(EGG_PATH):
    sys.path.append(EGG_PATH)
else:
    raise FileNotFoundError(f"Cannot find CARLA egg file at: {EGG_PATH}")

import carla

def test_connection():
    try:
        # Connect to localhost on default port 2000
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(5.0)
        
        world = client.get_world()
        carla_map = world.get_map()
        print(f" Successfully connected to CARLA! Map name: {carla_map.name}")
        
    except Exception as e:
        print(f" Connection failed: {e}")

if __name__ == '__main__':
    test_connection()