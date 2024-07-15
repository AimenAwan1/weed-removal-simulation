import xml.etree.ElementTree as ET
import os
from ament_index_python.packages import get_package_share_directory

def add_velocity_controller(content, namespace_arg, gazebo_pkg_arg):
    # Parse the XML content
    root = ET.fromstring(content)

    # Create the new element
    velocity_controller = ET.Element('xacro:velocity_controller')
    velocity_controller.set('namespace', namespace_arg)
    velocity_controller.set('gazebo_pkg', gazebo_pkg_arg)

    # Find the last child element of the "robot" tag
    root.append(velocity_controller)
    # Convert the modified XML tree back to a string
    modified_content = ET.tostring(root, encoding='unicode')
    
    modified_content = modified_content.replace("ns0", "xacro")
    return modified_content

ign_pkg = get_package_share_directory('name_ign')
xacro_file = os.path.join(ign_pkg,'robot','name.gazebo.xacro')

with open(xacro_file, "r") as file:
    content = file.read()
names = ['coconut1','coconut2','coconut3']
for name in names:
    content = add_velocity_controller(content, name, 'name_ign')

new_file_path = os.path.join(ign_pkg,'robot','control_name.gazebo.xacro')
with open(new_file_path, "w") as file:
    file.write(content)