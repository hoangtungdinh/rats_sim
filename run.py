"""
Scripts to bootstrap a rats show.
"""

import subprocess
import os
import atexit
import signal
import time

import shutil
import yaml
import glob
import sys
import datetime


def start():
    """
    Main entrance of the scripts.
    """
    main_config_file_name = sys.argv[1]
    # get the current path of the script
    current_path = os.path.dirname(os.path.realpath(__file__))
    # parse the main config file
    parsed_config_file = parse_yaml_file(current_path + '/' + main_config_file_name)
    # convert the parsed config file to python dictionary
    configs = read_yaml_file(parsed_config_file)

    # the list of all active processes
    tracker = {'processes': [], 'opened_files': []}
    # clean up on exit
    atexit.register(clean_up, tracker)

    # here we go
    print('Start the program...')

    log_dir_abs_path = os.path.expanduser(
        '~') + '/run_script_log/' + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + '/'

    start_bebops(configs['bebops'], tracker, log_dir_abs_path)
    start_synchronizer(configs['synchronizer'], tracker, log_dir_abs_path + 'synchronizer')

    # to keep the script alive
    input()


def start_bebops(bebop_configs, tracker, log_dir_abs_path):
    # iterate over all bebops
    for bebop, config in bebop_configs.items():
        # start a bebop using her own config
        start_single_bebop(tracker=tracker, config=config,
                           log_file_prefix_abs_path=log_dir_abs_path + bebop)


def read_yaml_file(yaml_file):
    """
    Reads a yaml file and translate it to a python dictionary.
    :param yaml_file: the absolute directory of the yaml file to be read
    :type yaml_file: str
    :return: the python dictionary representing the content of the yaml file
    :rtype: dict
    """
    with open(yaml_file, 'r') as stream:
        try:
            return yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)


def parse_yaml_file(yaml_file):
    """
    Parses a yaml file. A new temporary yaml file will be generated based on the input file with
    all the substitution arguments replaced by their values.
    :param yaml_file: the absolute directory of the yaml file to be parsed
    :type yaml_file: str
    :return: the absolute directory of the new temporary yaml file
    :rtype: str
    """
    file = open(yaml_file, 'r')
    content = file.read()
    file.close()

    # replace the absolute path variables
    content = content.replace('${abs_path}', os.path.dirname(os.path.realpath(__file__)))

    # read all other variables
    variables = read_yaml_file(os.path.dirname(os.path.realpath(__file__)) + '/variables.yaml')

    # replace all substitution arguments/variables by their values defined in variables.yaml
    for key, value in variables.items():
        content = content.replace('${' + key + '}', value)

    # check if there is any substitution argument not being defined in variables.yaml
    index = content.find('${')
    if index != -1:
        print('Cannot parse ' + yaml_file + '. Variable ' + content[index:content.find(
            '}') + 1] + ' is not defined in variables.yaml.')
        # if such variable exists, exit immediately
        exit()

    # the absolute directory of the temporary file
    parsed_file = yaml_file.replace('.yaml', '_tmp.yaml')

    # write the temporary file to disk
    tmp_file = open(parsed_file, 'w')
    tmp_file.write(content)
    tmp_file.close()

    # return the absolute directory of the temporary file
    return parsed_file


def start_single_bebop(tracker, config, log_file_prefix_abs_path):
    """
    Starts a single bebop.
    :param tracker: the list of active processes
    :type tracker: list
    :param config: the configuration of the bebop
    :type config: dict
    """
    my_env = create_env(config['gazebo_port'], config['ros_master_port'])
    launch_ros_master(my_env, config['ros_master_port'], tracker, config['master_sync_config_file'],
                      log_file_prefix_abs_path)
    launch_tum_sim(my_env, config['initial_position'], log_file_prefix_abs_path, tracker)
    launch_beswarm(my_env, tracker, config['beswarm_config'], log_file_prefix_abs_path)


def launch_tum_sim(my_env, initial_position, log_file_prefix_abs_path, tracker):
    launch_tum_sim_cmd = 'roslaunch cvg_sim_gazebo rats_sim.launch x:=' + initial_position[
        0] + ' y:=' + initial_position[1] + ' z:=' + initial_position[2] + ' yaw:=' + \
                         initial_position[3]
    execute_cmd(launch_tum_sim_cmd, my_env, log_file_prefix_abs_path + '_launch_tum_sim', tracker)
    time.sleep(5)
    turn_off_sim_time_cmd = 'rosparam set use_sim_time False'
    execute_cmd(turn_off_sim_time_cmd, my_env, log_file_prefix_abs_path + '_tum_sim_param', tracker)
    time.sleep(1)


def start_synchronizer(synchronizer_config, tracker, log_file_prefix_abs_path):
    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + synchronizer_config['ros_master_port']
    launch_ros_master(my_env, synchronizer_config['ros_master_port'], tracker,
                      synchronizer_config['master_sync_config_file'], log_file_prefix_abs_path)
    set_ros_parameters(my_env, tracker, synchronizer_config['rosparam'],
                       log_file_prefix_abs_path + '_set_rosparam.log')
    synchronizer_launch_cmd = 'rosrun rats ' + synchronizer_config['python_node']
    execute_cmd(synchronizer_launch_cmd, my_env, log_file_prefix_abs_path + '_launch_synchronizer',
                tracker)
    time.sleep(2)


def launch_beswarm(my_env, tracker, beswarm_config, log_file_prefix_abs_path):
    # delete build script folder
    build_script_dir = execute_cmd_and_get_output('rospack find rats') + '/BeSwarm/build/scripts'
    shutil.rmtree(build_script_dir, ignore_errors=True)
    # parse the beswarm config file and load it to the parameter server
    parsed_beswarm_config_file = parse_yaml_file(beswarm_config['beswarm_config_file'])
    load_param_cmd = 'rosparam load ' + parsed_beswarm_config_file
    execute_cmd(load_param_cmd, my_env, log_file_prefix_abs_path + '_rosparam_load.log', tracker)
    time.sleep(2)
    # set some remaining parameters to the parameter server
    set_ros_parameters(my_env, tracker, beswarm_config['rosparam'],
                       log_file_prefix_abs_path + '_set_rosparam.log')
    time.sleep(2)
    # launch the java node
    beswarm_launch_cmd = 'rosrun rats BeSwarm ' + beswarm_config['javanode'] + ' __name:=' + \
                         beswarm_config['nodename']
    execute_cmd(beswarm_launch_cmd, my_env, log_file_prefix_abs_path + '_launch_beswarm.log',
                tracker)
    time.sleep(2)


def set_ros_parameters(my_env, tracker, ros_params, log_file_abs_path):
    for key, value in ros_params.items():
        set_param_cmd = 'rosparam set ' + str(key) + ' ' + str(value)
        execute_cmd(set_param_cmd, my_env, log_file_abs_path, tracker)


def launch_ros_master(my_env, port, tracker, master_sync_config_file_abs_path,
                      log_file_prefix_abs_path):
    # start a ros master
    roscore_cmd = 'roscore -p ' + port
    execute_cmd(roscore_cmd, my_env, log_file_prefix_abs_path + '_roscore.log', tracker)
    time.sleep(2)
    # start master_discovery_fkie (to discover other ros masters)
    master_discovery_cmd = 'rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1'
    execute_cmd(master_discovery_cmd, my_env, log_file_prefix_abs_path + '_master_discovery.log',
                tracker)
    time.sleep(2)
    # start master_sync_fkie (to sync with other ros masters
    parsed_master_sync_config_file = parse_yaml_file(master_sync_config_file_abs_path)
    sync_cmd = 'rosrun master_sync_fkie master_sync _interface_url:=' + \
               parsed_master_sync_config_file
    execute_cmd(sync_cmd, my_env, log_file_prefix_abs_path + '_sync_cmd.log', tracker)
    time.sleep(2)


def create_env(gazebo_port, ros_master_port):
    my_env = os.environ.copy()
    my_env['GAZEBO_MASTER_URI'] = 'http://localhost:' + gazebo_port
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + ros_master_port
    return my_env


def clean_up(tracker):
    """
    Cleans up on exit.
    :param tracker: the list of active processes
    :type tracker: dict
    """
    # remove all generated tmp files
    remove_tmp_files()
    # terminate all processes
    terminate_all_processes(tracker['processes'])
    close_all_opened_files(tracker['opened_files'])


def terminate_all_processes(processes):
    """
    Terminates all processes.
    :param processes: the list of active processes
    :type processes: list
    """
    for p in processes:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except KeyboardInterrupt:
            pass
    print('cleaned up')


def close_all_opened_files(opened_files):
    for f in opened_files:
        f.flush()
        f.close()


def remove_tmp_files():
    """
    Removes all generated tmp files.
    """
    for file_name in glob.glob('./*_tmp.yaml'):
        os.remove(file_name)


def execute_cmd(cmd, my_env, log_file_abs_path, tracker):
    print(cmd)
    os.makedirs(os.path.dirname(log_file_abs_path), exist_ok=True)
    log_file = open(log_file_abs_path, 'a+')
    tracker['processes'].append(
        subprocess.Popen(cmd.split(), env=my_env, stdout=log_file, stderr=subprocess.STDOUT))
    tracker['opened_files'].append(log_file)


def execute_cmd_and_get_output(cmd):
    print(cmd)
    return subprocess.check_output(cmd.split()).decode("utf-8").replace('\n', '')


if __name__ == '__main__':
    start()
