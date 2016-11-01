import os
import atexit
import time

import datetime

from SwarmBootstrapUtils import clean_up
from SwarmBootstrapUtils import configs
from SwarmBootstrapUtils import executor


def start():
    config_dir = configs.get_config_dir()
    main_config = configs.get_main_config(config_dir)

    # the list of all active processes
    tracker = {'processes': [], 'opened_files': []}
    # clean up on exit
    atexit.register(clean_up.clean_up, tracker['processes'], tracker['opened_files'], config_dir)

    # here we go
    print('Start the program...')

    log_dir = os.path.expanduser('~') + '/log_rats_sim/' + datetime.datetime.now().strftime(
        "%Y-%m-%d-%H-%M-%S")

    start_bebops(main_config['bebops'], tracker, log_dir, config_dir)
    executor.start_synchronizer(main_config['synchronizer'], tracker, log_dir + '/synchronizer',
                                config_dir)

    # to keep the script alive
    input()


def start_bebops(bebop_configs, tracker, logdir, config_dir):
    executor.check_unique_integer_id(bebop_configs)
    # iterate over all bebops
    for bebop, config in bebop_configs.items():
        # start a bebop using her own config
        start_single_bebop(tracker=tracker, config=config, log_dir=logdir + '/' + bebop,
                           config_dir=config_dir)


def start_single_bebop(tracker, config, log_dir, config_dir):
    my_env = create_env(config['gazebo_port'], config['ros_master_port'])
    executor.launch_ros_master(my_env, config['ros_master_port'], tracker, config_dir, log_dir)
    launch_tum_sim(my_env, config['initial_position'], log_dir, tracker, config_dir)
    executor.launch_beswarm(my_env, tracker, config['beswarm_config'], config_dir, log_dir)


def launch_tum_sim(my_env, initial_position, log_dir, tracker, config_dir):
    launch_tum_sim_cmd = 'roslaunch ' + config_dir + '/rats_sim.launch x:=' + str(
        initial_position[0]) + ' y:=' + str(initial_position[1]) + ' z:=' + str(
        initial_position[2]) + ' yaw:=' + str(initial_position[3])
    executor.execute_cmd(launch_tum_sim_cmd, my_env, log_dir + '/launch_tum_sim.log', tracker)
    time.sleep(5)
    turn_off_sim_time_cmd = 'rosparam set use_sim_time False'
    executor.execute_cmd(turn_off_sim_time_cmd, my_env, log_dir + '/turn_off_sim_time.log', tracker)
    time.sleep(1)


def create_env(gazebo_port, ros_master_port):
    my_env = os.environ.copy()
    my_env['ROS_IP'] = '127.0.0.1'
    my_env['GAZEBO_MASTER_URI'] = 'http://localhost:' + gazebo_port
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + ros_master_port
    return my_env


if __name__ == '__main__':
    start()
