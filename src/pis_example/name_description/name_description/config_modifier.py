import yaml, os
    
def modify_rviz_config(package,rviz_file,namespace):
    rviz_path = os.path.join(package,'config',rviz_file)
    with open(rviz_path,'r') as file:
        content = yaml.load(file,Loader=yaml.SafeLoader)
    new_content = content.copy()
    
    default_frame = content['Visualization Manager']['Global Options']['Fixed Frame'] 
    new_content['Visualization Manager']['Global Options']['Fixed Frame'] = namespace+'/'+default_frame
    
    idx = 0
    for display in content['Visualization Manager']['Displays']:
        if display['Class'] == 'rviz_default_plugins/RobotModel':
            default_robot_description = display['Description Topic']['Value']
            new_content['Visualization Manager']['Displays'][idx]['Description Topic']['Value'] = namespace+default_robot_description
            new_content['Visualization Manager']['Displays'][idx]['TF Prefix'] = namespace
            break
        idx = idx + 1 
    new_rviz_path = os.path.join(package,'config',namespace+'_config.rviz')
    with open(new_rviz_path,'w') as file:
        yaml.dump(new_content,file)
    return new_rviz_path