import xml.etree.ElementTree as ET

def add_velocity_controller(content, namespace_arg, gazebo_pkg_arg,is_fleet=False):
    # Parse the XML content
    root = ET.fromstring(content)

    # Create the new element
    velocity_controller = ET.Element('xacro:velocity_controller')
    velocity_controller.set('namespace', namespace_arg)
    velocity_controller.set('gazebo_pkg', gazebo_pkg_arg)
    if is_fleet:
        is_fleet = ''
    else:
        is_fleet = 'false'
    velocity_controller.set('is_fleet', is_fleet)
    # Find the last child element of the "robot" tag
    root.append(velocity_controller)
    # Convert the modified XML tree back to a string
    modified_content = ET.tostring(root, encoding='unicode')
    
    modified_content = modified_content.replace("ns0", "xacro")
    return modified_content

def append_XML(content,tag:str,arguments:dict):
    root = ET.fromstring(content)
    tag_element = ET.Element(tag)
    for arg,value in arguments.items():
        tag_element.set(arg,value)
    root.append(tag_element)
    modified_content = ET.tostring(root, encoding='unicode')

    modified_content = modified_content.replace("ns0", "xacro")
    return modified_content

def add_robot(content:str,description_pkg:str,gazebo_pkg:str,namespace:str,p0:list=None,r0:list=None):
    if p0:
        p0_str = ['{:.10f}'.format(x) for x in p0]
    else:
        p0_str = '0.0 0.0 0.0'
    if r0:
        r0_str = ['{:.10f}'.format(x) for x in r0]
    else:
        r0_str = '0.0 0.0 0.0'
       
    robot_arguments = {
        "description_pkg":description_pkg,
        "gazebo_pkg":gazebo_pkg,
        "namespace":namespace
    }
    ground_truth_arguments = {
        "namespace":namespace,
        "p0":p0_str,
        "r0":r0_str,
    }
    modified_content = append_XML(content,"xacro:robot_with_plugins",robot_arguments)
    modified_content = append_XML(modified_content,"xacro:ground_truth",ground_truth_arguments)

    return modified_content

def add_controller_plugin(content,gazebo_pkg:str):
    plugins_arguments = {
        "gazebo_pkg":gazebo_pkg,
    }
    modified_content = append_XML(content,"xacro:controller_plugins",plugins_arguments)

    return modified_content
