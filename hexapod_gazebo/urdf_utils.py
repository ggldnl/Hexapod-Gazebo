"""
Provides prepare_urdf(), which takes the raw URDF from the Hexapod-Hardware
submodule and returns a patched XML string ready for Gazebo:

  1. Mesh filenames are rewritten from relative paths to absolute file:// URIs.
  2. All revolute joints are renamed by appending '_joint' to avoid collisions
     with link names in Gazebo's SDF frame graph.
  3. A <ros2_control> block is injected to expose every non-fixed joint as a
     position-controlled actuator via gz_ros2_control.
  4. A <gazebo> plugin block is injected to load the gz_ros2_control system
     plugin and point it at the controllers YAML.
"""

import xml.etree.ElementTree as ET
import yaml
import os


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

    _fix_mesh_paths(root, hardware_dir)
    _rename_revolute_joints(root)
    _inject_ros2_control(root)
    _inject_gazebo_plugin(root, config_path)

    xml_string = '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode')
    return xml_string


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