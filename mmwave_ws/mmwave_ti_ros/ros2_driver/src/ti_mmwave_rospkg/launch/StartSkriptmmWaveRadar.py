import subprocess
import time
import os

current_dir = os.path.dirname(os.path.abspath(__file__))

# Start first radar launch file
launch1 = subprocess.Popen([
    'ros2', 'launch', os.path.join(current_dir, '6843AOP_Multi_0.py')
])
time.sleep(5.0)  # reduced delay for efficiency

# Start second radar launch file
launch2 = subprocess.Popen([
    'ros2', 'launch', os.path.join(current_dir, '6843AOP_Multi_1.py')
])
time.sleep(5.0)  # reduced delay for efficiency

# Start third radar launch file
launch3 = subprocess.Popen([
    'ros2', 'launch', os.path.join(current_dir, '6843AOP_Multi_2.py')
])
time.sleep(5.0)  # reduced delay for efficiency

# Start fourth radar launch file
launch4 = subprocess.Popen([
    'ros2', 'launch', os.path.join(current_dir, '6843AOP_Multi_3.py')
])

# Wait until all launch files have terminated
launch1.wait()
launch2.wait()
launch3.wait()
launch4.wait()
