import time
import cflib.crtp
import numpy as np
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.Ori_CF.multi_agent_task_allocation.src.CF_Trajectory import Generate_Trajectory, upload_trajectory
import params

class Flight_manager(object):
    def __init__(self, drone_num):
        self.drone_num = drone_num
        self.uri_list = params.uri_list[:self.drone_num]
        self.base = params.base
        self.uri_dict = dict()
        self.reversed_uri_dict = dict()
        for i  in range(len(self.uri_list)):
            self.uri_dict[i] = self.uri_list[i]
            self.reversed_uri_dict[self.uri_list[i]] = i
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        self.swarm = Swarm(set(self.uri_list), factory=factory)
        self.swarm.open_links()
        self.swarm.parallel_safe(self.activate_high_level_commander)
        self.swarm.reset_estimators()
        self.open_threads = [[]] * self.drone_num
        self.sleep_time = params.sleep_time
        self.smooth_points_num = params.segments_num + 1
        self.max_dist2target = params.dist_to_target
        self.max_dist2base = params.dist_to_base
        self.velocity = params.linear_velocity
       
    def activate_high_level_commander(self, scf):
        scf.cf.param.set_value('commander.enHighLevel', '1')

    def wait_thread_killed(self, drone_idx):
        """
        wait until all thread are killed (drone_idx = 'all')
        wait until specific thread is killed(e.g. drone_idx = 1)
        """
        if drone_idx == 'all':
            thread_alive = True
            while thread_alive:
                thread_alive = False
                for thread in self.open_threads:
                    if thread.is_alive():
                        thread_alive = True
                time.sleep(0.05)
                # print('-- waiting until all threads are killed')
            print('all threads dead') 
        else:
            while self.open_threads[drone_idx].is_alive():
                time.sleep(0.05)
                print(f'waiting until thread {drone_idx} is killed')
            print(f'thread {drone_idx} is killed') 


    def _take_off(self, scf):
        cf = scf.cf
        commander = cf.high_level_commander
        commander.takeoff(params.take_off_height, 2.0)
        time.sleep(2.0) 
    
    def take_off_swarm(self):
        threads = self.swarm.parallel_safe(self._take_off)
        for i in range(len(threads)):
            self.open_threads[i] = threads[i]
        self.wait_thread_killed('all')
        threads = self.swarm.parallel_safe(self._go_to_base)
        for i in range(len(threads)):
            self.open_threads[i] = threads[i]
        self.wait_thread_killed('all')

    def _go_to_base(self, scf):
        cf = scf.cf
        commander = cf.high_level_commander
        x,y,z = self.base[self.reversed_uri_dict[scf.cf.link_uri]]
        commander.go_to(x, y, z, yaw=0, duration_s=2)
        time.sleep(2)
    
    
    def _go_to(self, scf, goal, drone_idx):
        cf = scf.cf
        commander = cf.high_level_commander 
        x,y,z = goal
        commander.go_to(x, y, z, yaw=0, duration_s=0.1)


    def go_to(self, drone_idx, goal):
        thread = self.swarm.daemon_process(self._go_to, self.uri_dict[drone_idx], [goal, drone_idx])
        self.open_threads[drone_idx] = thread


    def _execute_trajectory(self, scf, waypoints, drone_idx): 
        cf = scf.cf
        commander = cf.high_level_commander 
        if len(waypoints) > self.smooth_points_num:
            waypoints1 = waypoints[:self.smooth_points_num] # target to retreat
            waypoints2 = waypoints[self.smooth_points_num:] # retreat to target
            wp_list = [waypoints1, waypoints2]
            velocity = 0.8
        else:
            wp_list = [waypoints]
            velocity = 1.5
        try:
            for waypoints in wp_list:
                x, y, z = waypoints[0]
                commander.go_to(x, y, z, yaw=0, duration_s=0.1)
                trajectory_id = 1
                traj = Generate_Trajectory(waypoints, velocity=velocity, plotting=0, force_zero_yaw=False, is_smoothen=True)
                traj_coef = traj.poly_coef
                duration = upload_trajectory(cf, trajectory_id ,traj_coef)
                commander.start_trajectory(trajectory_id, 1.0, False)
                time.sleep(0.2)
        except:
            print('failed to execute trajectory')

    
    def execute_trajectory_mt(self, drone_idx, waypoints):# send trajectory with multi thread mode
        thread = self.swarm.daemon_process(self._execute_trajectory, self.uri_dict[drone_idx], [waypoints, drone_idx])
        self.open_threads[drone_idx] = thread
    
    def get_position(self, drone_idx):
        scf = self.swarm._cfs[self.uri_dict[drone_idx]]
        self.swarm.get_single_cf_estimated_position(scf)
        # self.swarm.daemon_process(self.swarm.get_single_cf_estimated_position, self.uri_dict[drone_idx])
        return self.swarm._positions[self.uri_dict[drone_idx]]
    
    def get_battery(self, drone_idx):
        scf = self.swarm._cfs[self.uri_dict[drone_idx]]
        self.swarm.get_battert_voltage(scf)
        return self.swarm.battery[self.uri_dict[drone_idx]]        


        
    def reached_goal(self, drone_idx, goal, title='target'):
        try:
            self.get_position(drone_idx)
            current_x, current_y, current_z = self.swarm._positions[self.uri_dict[drone_idx]]
            dist2goal = ((current_x - goal[0])**2 + (current_y - goal[1])**2 +(current_z - goal[2])**2 )**0.5
            max_dist = self.max_dist2target if title == 'target' else self.max_dist2base
            if dist2goal < max_dist:
                print(f'drone {drone_idx} reached goal')
                return 1
            else:
                return 0
        except:
            return 0

    def _land(self, scf, drone_idx):
        self.wait_thread_killed(drone_idx)
        cf = scf.cf
        commander = cf.high_level_commander
        commander.land(0.0, 4.0)
        time.sleep(4)
        commander.stop()
    
    def land(self, drone_idx, drones=None): 
        if drone_idx == 'all':
            for i in range(len(drones)):
                if drones[i].is_active:
                    thread = self.swarm.daemon_process(self._land, self.uri_dict[i], [i])
                    self.open_threads[i] = thread   
            self.wait_thread_killed(drone_idx)     
            self.swarm.close_links()
        else:
            thread = self.swarm.daemon_process(self._land, self.uri_dict[drone_idx], [drone_idx])
            self.open_threads[drone_idx] = thread 
        

    
    def sleep(self):
        time.sleep(self.sleep_time)




