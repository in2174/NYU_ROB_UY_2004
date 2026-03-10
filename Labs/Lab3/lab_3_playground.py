
import numpy as np
import scipy
np.set_printoptions(precision=3, suppress=True)
from matplotlib import pyplot as plt

def rotation_x(angle):
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(angle), -np.sin(angle), 0],
        [0, np.sin(angle), np.cos(angle), 0],
        [0, 0, 0, 1]
    ])

def rotation_y(angle):
    return np.array([
        [np.cos(angle), 0, np.sin(angle), 0],
        [0, 1, 0, 0],
        [-np.sin(angle), 0, np.cos(angle), 0],
        [0, 0, 0, 1]
    ])

def rotation_z(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0, 0],
        [np.sin(angle), np.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def translation(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

class InverseKinematics():

    def __init__(self):
        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None
        self.counter = 0

        # Trotting gate positions
        ################################################################################################
        # TODO: Implement the trotting gait
        ################################################################################################
        # forward waypoints
        touch_down_position = np.array([-0.07,0,-0.14]) # change -0.05 to -0.07 for faster 
        stand_position_1 = np.array([-0.025,0,-0.14])
        stand_position_2 = np.array([0,0,-0.14])
        stand_position_3 = np.array([0.025,0,-0.14])
        liftoff_position = np.array([0.07,0,-0.14]) # change 0.05 to 0.07 for faster 
        mid_swing_position = np.array([0,0,-0.03])  # change -0.05 to -0.03 for faster 
        
        ## trotting
        # TODO: Implement each leg’s trajectory in the trotting gait.
        rf_ee_offset = np.array([0.06, -0.09, 0])
        rf_ee_triangle_positions = np.array([
            ################################################################################################
            # TODO: Implement the trotting gait
            ################################################################################################
            touch_down_position, mid_swing_position, liftoff_position, stand_position_3, stand_position_2,
            stand_position_1
        ]) + rf_ee_offset
        
        lf_ee_offset = np.array([0.06, 0.09, 0])
        lf_ee_triangle_positions = np.array([
            ################################################################################################
            # TODO: Implement the trotting gait
            ################################################################################################
            touch_down_position, mid_swing_position, liftoff_position, stand_position_3, stand_position_2,
            stand_position_1
        ]) + lf_ee_offset
        
        rb_ee_offset = np.array([-0.11, -0.09, 0])
        rb_ee_triangle_positions = np.array([
            ################################################################################################
            # TODO: Implement the trotting gait
            ################################################################################################
            touch_down_position, mid_swing_position, liftoff_position, stand_position_3, stand_position_2,
            stand_position_1
        ]) + rb_ee_offset
        
        lb_ee_offset = np.array([-0.11, 0.09, 0])
        lb_ee_triangle_positions = np.array([
            ################################################################################################
            # TODO: Implement the trotting gait
            ################################################################################################
            touch_down_position, mid_swing_position, liftoff_position, stand_position_3, stand_position_2,
            stand_position_1

        ]) + lb_ee_offset

        self.fk_functions = [self.fr_leg_fk, self.fl_leg_fk, self.br_leg_fk, self.bl_leg_fk]
        
        self.ee_triangle_positions = [rf_ee_triangle_positions, lf_ee_triangle_positions, rb_ee_triangle_positions, lb_ee_triangle_positions]
        # self.target_joint_positions_cache, self.target_ee_cache = self.cache_target_joint_positions()
        
        # rotate right waypoints 
        # Right legs move +x, left legs move -x → robot spins in place
        # td_rot = np.array([-0.05, 0, -0.14])
        # s1_rot = np.array([-0.025, 0, -0.14])
        # s2_rot = np.array([0, 0, -0.14])
        # s3_rot = np.array([0.025, 0, -0.14])
        # lo_rot = np.array([0.05, 0, -0.14])
        # sw_rot = np.array([0, 0, -0.03])
        
        # flip x for left legs
        # flip = np.array([-1, 1, 1])  
        
        # rf_rot_positions = np.array([td_rot,        s1_rot,        s2_rot,        s3_rot,        lo_rot,        sw_rot       ]) + rf_ee_offset
        # lf_rot_positions = np.array([td_rot * flip, sw_rot,        lo_rot * flip, s3_rot * flip, s2_rot * flip, s1_rot * flip]) + lf_ee_offset
        # rb_rot_positions = np.array([td_rot,        s1_rot,        s2_rot,        s3_rot,        lo_rot,        sw_rot       ]) + rb_ee_offset
        # lb_rot_positions = np.array([td_rot * flip, sw_rot,        lo_rot * flip, s3_rot * flip, s2_rot * flip, s1_rot * flip]) + lb_ee_offset


        # Cache rotation gait by swapping ee_triangle_positions
        # self.ee_triangle_positions = [rf_rot_positions, lf_rot_positions, rb_rot_positions, lb_rot_positions]
        # self.rot_joint_positions_cache, self.rot_ee_cache = self.cache_target_joint_positions()
        # print(f'shape of target_joint_positions_cache: {self.target_joint_positions_cache.shape}')
        # print(f'shape of target_ee_cache: {self.target_ee_cache.shape}')


        ##### TODO: Uncomment when you want to generate the cache #######
        self.target_joint_positions_cache, self.target_ee_cache = self.cache_target_joint_positions()
        
        print(f'shape of target_joint_positions_cache: {self.target_joint_positions_cache.shape}')
        print(f'shape of target_ee_cache: {self.target_ee_cache.shape}')

        # Square path stuff
        # self.STEPS_FORWARD = 150  # tune for one side of square
        # self.STEPS_TURN    = 100  # tune for 90 degree turn
        # self.gait_phase = 'forward'
        # self.phase_step = 0

        # self.pd_timer_period = 1.0 / 200
        # self.ik_timer_period = 1.0 / 100
        # self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        # self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)

    def fr_leg_fk(self, theta):
        T_RF_0_1 = translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def fl_leg_fk(self, theta):
        T_LF_0_1 = translation(0.07500, 0.08350, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
        T_LF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_LF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-theta[2])
        T_LF_3_ee = translation(0.06231, -0.06216, -0.01800)
        T_LF_0_ee = T_LF_0_1 @ T_LF_1_2 @ T_LF_2_3 @ T_LF_3_ee
        return T_LF_0_ee[:3, 3]

    def br_leg_fk(self, theta):
        T_RB_0_1 = translation(-0.07500, -0.07250, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RB_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RB_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RB_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RB_0_ee = T_RB_0_1 @ T_RB_1_2 @ T_RB_2_3 @ T_RB_3_ee
        return T_RB_0_ee[:3, 3]

    def bl_leg_fk(self, theta):
        T_LB_0_1 = translation(-0.07500, 0.07250, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
        T_LB_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_LB_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-theta[2])
        T_LB_3_ee = translation(0.06231, -0.06216, -0.01800)
        T_LB_0_ee = T_LB_0_1 @ T_LB_1_2 @ T_LB_2_3 @ T_LB_3_ee
        return T_LB_0_ee[:3, 3]

    def forward_kinematics(self, theta):
        return np.concatenate([self.fk_functions[i](theta[3*i: 3*i+3]) for i in range(4)])
        
    def get_error_leg(self, theta, desired_position):
        ################################################################################################
        # TODO: [already done] paste lab 3 inverse kinematics here
        x = self.leg_forward_kinematics(theta)
        error = desired_position - x
        
        ################################################################################################
        return error.dot(error)

    def inverse_kinematics_single_leg(self, target_ee, leg_index, initial_guess=[0, 0, 0]):
        self.leg_forward_kinematics = self.fk_functions[leg_index]
        ################################################################################################
        # TODO: implement interpolation for all 4 legs here
        res = scipy.optimize.minimize(self.get_error_leg, initial_guess, args = (target_ee,))

        ################################################################################################
        return res.x

    def interpolate_triangle(self, t, leg_index):
        ################################################################################################
        # TODO: implement interpolation for all 4 legs here
        ################################################################################################
        pts = self.ee_triangle_positions[leg_index] 
        n = pts.shape[0]  # 6 (b/c we have 6 waypoints)                             

        t = t % 1.0    # makes sure t is in [0,1)
        s = t * n      # scale t to [0,6) because we have 6 segments
        i = int(s)     # which segment we're in (0 to 5)
        u = s - i      # how into that segment we are

        p0 = pts[i]             # start of current segment 
        p1 = pts[(i + 1) % n]   # end of current segment, wraps back to 0
        return (1 - u) * p0 + u * p1  # linear interpolation between p0 and p1 (weighted average)
        

    def cache_target_joint_positions(self):
        # Calculate and store the tarjoint positions for a cycle and all 4 legs
        target_joint_positions_cache = []
        target_ee_cache = []
        for leg_index in range(4):
            target_joint_positions_cache.append([])
            target_ee_cache.append([])
            target_joint_positions = [0] * 3
            for t in np.arange(0, 1, 0.02):
                target_ee = self.interpolate_triangle(t, leg_index)
                target_joint_positions = self.inverse_kinematics_single_leg(target_ee, leg_index, initial_guess=target_joint_positions)
                target_joint_positions_cache[leg_index].append(target_joint_positions)
                target_ee_cache[leg_index].append(target_ee)

        # (4, 50, 3) -> (50, 12)
        target_joint_positions_cache = np.concatenate(target_joint_positions_cache, axis=1)
        target_ee_cache = np.concatenate(target_ee_cache, axis=1)
        
        return target_joint_positions_cache, target_ee_cache

    def get_target_joint_positions(self):
        target_joint_positions = self.target_joint_positions_cache[self.counter]
        target_ee = self.target_ee_cache[self.counter]
        self.counter += 1
        if self.counter >= self.target_joint_positions_cache.shape[0]:
            self.counter = 0
        return target_ee, target_joint_positions

    # for square path
    # def get_target_joint_positions(self):
    #     # Pick cache based on current gait phase
    #     if self.gait_phase == 'forward':
    #         joint_pos = self.target_joint_positions_cache[self.counter]
    #         ee        = self.target_ee_cache[self.counter]
    #     else:
    #         joint_pos = self.rot_joint_positions_cache[self.counter]
    #         ee        = self.rot_ee_cache[self.counter]

    #     self.counter += 1
    #     if self.counter >= self.target_joint_positions_cache.shape[0]:
    #         self.counter = 0

    #     # Switch gait for square path
    #     self.phase_step += 1
    #     if self.gait_phase == 'forward' and self.phase_step >= self.STEPS_FORWARD:
    #         self.gait_phase = 'turn'
    #         self.phase_step = 0
    #     elif self.gait_phase == 'turn' and self.phase_step >= self.STEPS_TURN:
    #         self.gait_phase = 'forward'
    #         self.phase_step = 0

    #     return ee, joint_pos



def main():

    # Create an instance of the IK class
    inverse_kinematics = InverseKinematics()

    # Test IK solver on right front leg
    target_ee_list = [
        [0.11, -0.09, -0.14],
        [0.10, -0.09, -0.14],
        [0.09, -0.09, -0.14],
        [0.08, -0.09, -0.14],
        [0.07, -0.09, -0.14],
        [0.06, -0.09, -0.14],
        [0.05, -0.09, -0.14],
        [0.04, -0.09, -0.14],
        [0.03, -0.09, -0.14],
        [0.02, -0.09, -0.14],
        [0.01, -0.09, -0.14],
    ]

    inverse_kinematics.leg_forward_kinematics = inverse_kinematics.fk_functions[0]
    result_ee_list = []
    for target_ee in target_ee_list:
        theta = inverse_kinematics.inverse_kinematics_single_leg(target_ee, leg_index=0, initial_guess=[0, 0, 0])
        result_ee = inverse_kinematics.leg_forward_kinematics(theta)
        result_ee_list.append(result_ee)

    # Plot the EE results
    if len(result_ee_list) > 0:
        plt.plot(np.array(target_ee_list)[:,0],'k')
        plt.plot(np.array(result_ee_list)[:,0],'ro')
        plt.xlabel('Step')
        plt.ylabel('X (m)')
        plt.legend(['Target EE Position','Result EE Position'])
        plt.title('End Effector X position')
        plt.savefig('plot2faster')

    # Plot the cached trot gait path for one foot.
    if len(inverse_kinematics.target_ee_cache):
        x_list = []
        z_list = []
        for position in inverse_kinematics.target_ee_cache:
            x_list.append(position[0])
            z_list.append(position[2])

        x_list.append(x_list[0])
        z_list.append(z_list[0])
        plt.figure()

        plt.xlabel('X(m)')
        plt.ylabel('Z(m)')
        plt.title('EE front right foot trot gait')
        plt.plot(x_list, z_list)
        plt.savefig('plot3faster')



if __name__ == '__main__':
    main()
