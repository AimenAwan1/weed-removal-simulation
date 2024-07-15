import yaml, os
    
def modify_controller_config(package,controller_file,namespace):
    controller_path = os.path.join(package,'config',controller_file)
    with open(controller_path,'r') as file:
        content = yaml.load(file,Loader=yaml.SafeLoader)
    new_content = {}
    new_content[namespace] = {'controller_manager':content['controller_manager'].copy()} 
    controller_parameters = new_content[namespace]['controller_manager']['ros__parameters'].copy()
    default_controller_parameters = content['controller_manager']['ros__parameters']
    for key,value in default_controller_parameters.items():
        if not (key == 'update_rate'):
            controller_content = value.copy()
            del controller_parameters[key]
            controller_parameters[namespace+'/'+key] = controller_content
    new_content[namespace]['velocity_controllers'] = content['velocity_controllers']
    new_content[namespace]['effort_controllers'] = content['effort_controllers']
    new_controller_path = os.path.join(package,'config',namespace+'_controller_config.yaml')    
    with open(new_controller_path,'w') as file:
        yaml.dump(new_content,file)
    return new_controller_path
