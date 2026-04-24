from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'hexapod_gazebo'

def collect_data_files():
    """
    Build the data_files list for everything that needs to land in the share
    directory at install time. ament_python does not recurse automatically, so
    we walk Hexapod-Hardware ourselves and mirror the directory tree.
    """
    data = []

    # Standard ROS2 package metadata
    data.append((os.path.join('share', 'ament_index', 'resource_index', 'packages'),
                 [os.path.join('resource', package_name)]))
    data.append((os.path.join('share', package_name), ['package.xml']))
    data.append((os.path.join('share', package_name, 'launch'), glob("launch/*.py")))

    # Worlds and config
    data.append((os.path.join('share', package_name, 'worlds'), glob('worlds/*')))
    data.append((os.path.join('share', package_name, 'config'), glob('config/*')))

    # Hexapod-Hardware submodule: walk the full tree so every STL mesh and
    # the URDF files are available to Gazebo at runtime via absolute paths.
    for dirpath, dirnames, filenames in os.walk('Hexapod-Hardware'):
        # Skip hidden directories (e.g. .git)
        dirnames[:] = [d for d in dirnames if not d.startswith('.')]
        if not filenames:
            continue
        install_dir = os.path.join('share', package_name, dirpath)
        files = [os.path.join(dirpath, f) for f in filenames]
        data.append((install_dir, files))

    return data

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=collect_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='danielgigliotti99.dg@gmail.com',
    description='Gazebo mirror for the hexapod robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"joint_command_bridge = {package_name}.joint_command_bridge:main",
            f"joy_teleop = {package_name}.joy_teleop_node:main",
        ],
    },
)
