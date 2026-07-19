"""
Provides prepare_urdf(), which takes the raw URDF from the Hexapod-Hardware
submodule and returns a patched XML string ready for Gazebo:

  1. Collision meshes are replaced by axis-aligned boxes, and links that never
     touch anything lose their collision element.
  2. Mesh filenames are rewritten from relative paths to absolute file:// URIs.
  3. All revolute joints are renamed by appending '_joint' to avoid collisions
     with link names in Gazebo's SDF frame graph.
  4. A <ros2_control> block is injected to expose every non-fixed joint as a
     position-controlled actuator via gz_ros2_control.
  5. A <gazebo> plugin block is injected to load the gz_ros2_control system
     plugin and point it at the controllers YAML.
"""

import math
import struct
import xml.etree.ElementTree as ET
import yaml
import os


# Meshes that keep a collision proxy: the printed structure, the feet and the
# battery, i.e. the parts that can actually reach the ground. Everything else is
# internal (servo bodies, the electronics boards, screws, heat-set inserts) and
# loses its collision element entirely
COLLIDING_MESHES = frozenset({
    'base',
    'top_stiffeners',
    'bottom_stiffeners',
    'battery',
    'battery_mount',
    'femur',
    'tibia',
    'bracket_A',
    'bracket_B',
})

# Gazebo rejects a box with a zero-length side, so flat plates get a floor
MIN_BOX_SIDE = 1e-4


def parse_urdf(urdf_path: str, hardware_dir: str, config_path: str = None):
    """
    Load, patch and return (urdf_string, joint_names).

    urdf_path = absolute path to the .urdf file
    hardware_dir = absolute path to the Hexapod-Hardware directory
    controller_yaml_path = absolute path where the controllers YAML will be
                            written (must match what the Gazebo plugin loads)
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Runs first, while mesh filenames are still relative to hardware_dir
    boxes, dropped = _simplify_collisions(root, hardware_dir)
    print(f'[urdf_utils] collisions: {boxes} meshes boxed, {dropped} dropped')

    _fix_mesh_paths(root, hardware_dir)
    _rename_revolute_joints(root)
    _inject_ros2_control(root)
    _inject_gazebo_plugin(root, config_path)

    xml_string = '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode')
    return xml_string


def _simplify_collisions(root: ET.Element, hardware_dir: str) -> tuple[int, int]:
    """
    Swap every collision mesh for an axis-aligned box taken from the STL, and
    drop the collision element from links that never touch anything.

    The URDF ships the full CAD mesh as collision geometry, which puts roughly
    517k triangles into the physics scene and pins Gazebo's real-time factor far
    below 1.0. That matters beyond being slow: gz_ros2_control steps the
    controller manager on sim time, so a low RTF silently decimates the incoming
    joint commands and the gait comes apart. Boxes cost nothing to collide.

    Returns (boxed, dropped).
    """
    cache: dict[str, tuple] = {}
    boxed = dropped = 0

    for link in root.iter('link'):
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                continue

            filename = mesh.get('filename', '')
            name = os.path.splitext(os.path.basename(filename))[0]
            if name not in COLLIDING_MESHES:
                link.remove(collision)
                dropped += 1
                continue

            if filename not in cache:
                cache[filename] = _stl_aabb(
                    os.path.join(hardware_dir, filename.lstrip('./')))
            size, center = cache[filename]

            # The STLs are in millimetres, so the mesh scale applies to both the
            # extent and the offset of the centre
            scale = [float(v) for v in mesh.get('scale', '1 1 1').split()]
            size = [max(s * k, MIN_BOX_SIDE) for s, k in zip(size, scale)]
            center = [c * k for c, k in zip(center, scale)]

            _replace_with_box(collision, size, center)
            boxed += 1

    return boxed, dropped


def _replace_with_box(collision: ET.Element, size, center) -> None:
    """
    Put a box of `size` where the mesh was.

    The mesh was placed in the link frame by the collision origin, but a box is
    centred on itself, so the mesh's bounding-box centre has to be rotated by
    that origin's rpy and added to its xyz. Nearly every collision in this URDF
    carries a non-trivial rpy, so skipping the rotation would scatter the boxes.
    """
    geometry = collision.find('geometry')
    geometry.remove(geometry.find('mesh'))
    ET.SubElement(geometry, 'box').set('size', ' '.join(f'{v:.6g}' for v in size))

    origin = collision.find('origin')
    if origin is None:
        origin = ET.SubElement(collision, 'origin')
    xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]

    offset = _rotate(rpy, center)
    origin.set('xyz', ' '.join(f'{a + b:.6g}' for a, b in zip(xyz, offset)))
    origin.set('rpy', ' '.join(f'{v:.6g}' for v in rpy))


def _rotate(rpy, v):
    """Rotate v by a URDF rpy triple (fixed-axis XYZ, so Rz @ Ry @ Rx)."""
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    m = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp,     cp * sr,                cp * cr),
    )
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def _stl_aabb(path: str):
    """
    Return (size, centre) of an STL's axis-aligned bounding box, in the units the
    file is written in.
    """
    with open(path, 'rb') as f:
        data = f.read()

    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for vertex in _stl_vertices(data):
        for i, value in enumerate(vertex):
            lo[i] = min(lo[i], value)
            hi[i] = max(hi[i], value)

    if lo[0] > hi[0]:
        raise RuntimeError(f'no vertices found in {path}')

    return ([hi[i] - lo[i] for i in range(3)],
            [(hi[i] + lo[i]) / 2.0 for i in range(3)])


def _stl_vertices(data: bytes):
    """
    Yield every vertex of an STL blob.

    Binary is detected by the file length matching the triangle count in the
    header, which is the only reliable test: a binary STL may also start with
    the word 'solid'.
    """
    if len(data) >= 84:
        count = struct.unpack('<I', data[80:84])[0]
        if len(data) == 84 + 50 * count:
            for tri in struct.iter_unpack('<12fH', data[84:]):
                yield tri[3], tri[4], tri[5]
                yield tri[6], tri[7], tri[8]
                yield tri[9], tri[10], tri[11]
            return

    for line in data.decode('ascii', 'replace').splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == 'vertex':
            yield float(parts[1]), float(parts[2]), float(parts[3])


def _fix_mesh_paths(root: ET.Element, hardware_dir: str) -> None:
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename', '')
        if not filename:
            continue
        if filename.startswith('file://') or filename.startswith('package://'):
            continue
        clean = filename.lstrip('./')
        mesh.set('filename', 'file://' + os.path.join(hardware_dir, clean))


def _rename_revolute_joints(root: ET.Element) -> dict:
    """
    Append '_joint' to every revolute joint name to avoid collisions with
    link names in Gazebo's SDF frame graph.
    """

    rename_map = {}
    for joint in root.iter('joint'):
        if joint.get('type', '') == 'revolute':
            original = joint.get('name', '')
            if original and not original.endswith('_joint'):
                new_name = original + '_joint'
                joint.set('name', new_name)
                rename_map[original] = new_name

    return rename_map


def _inject_ros2_control(root: ET.Element) -> list[str]:
    """
    Inject a <ros2_control> block for every non-fixed joint.
    Joint names are read from the tree after renaming, so they are always
    consistent with what Gazebo will see.
    Returns the ordered list of controllable joint names.
    """
    controllable_joints = [
        joint.get('name')
        for joint in root.iter('joint')
        if joint.get('type', 'fixed') != 'fixed' and joint.get('name')
    ]

    if not controllable_joints:
        raise RuntimeError(
            'No non-fixed joints found in the URDF. '
            'Check that the correct URDF file is being loaded.'
        )

    rc = ET.SubElement(root, 'ros2_control')
    rc.set('name', 'GazeboSimSystem')
    rc.set('type', 'system')

    hw = ET.SubElement(rc, 'hardware')
    ET.SubElement(hw, 'plugin').text = 'gz_ros2_control/GazeboSimSystem'

    for joint_name in controllable_joints:
        j_elem = ET.SubElement(rc, 'joint')
        j_elem.set('name', joint_name)
        ET.SubElement(j_elem, 'command_interface').set('name', 'position')
        si_pos = ET.SubElement(j_elem, 'state_interface')
        si_pos.set('name', 'position')
        ET.SubElement(si_pos, 'param', {'name': 'initial_value'}).text = '0.0'
        ET.SubElement(j_elem, 'state_interface').set('name', 'velocity')

    return controllable_joints


def _inject_gazebo_plugin(root: ET.Element, controllers_yaml_path: str) -> None:
    """
    Inject the <gazebo> plugin block that tells Gazebo to load the
    gz_ros2_control system plugin. Without this, controller_manager
    never starts regardless of the <ros2_control> block.
    """
    gazebo = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin')
    plugin.set('filename', 'gz_ros2_control-system')
    plugin.set('name', 'gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin, 'parameters').text = controllers_yaml_path
    ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'namespace').text = '/'