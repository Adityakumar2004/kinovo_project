#!/usr/bin/python3
import rospy
import rospy, numpy as np
import kortex_driver.msg as msg
from std_msgs.msg import String
from moveit_commander import RobotCommander, MoveGroupCommander
# from visual_servoing_1_uw import twist_tf_camera_ef

import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft
from kortex_examples.msg import Jacobian

class velocity_command():
    def __init__(self):
        #-------------------------------------------------------------------------------------------------
        # Initialising the rosnode
        rospy.init_node('jacknode', anonymous=True)
        self.robot = RobotCommander('/my_gen3/robot_description', '/my_gen3')
        self.group = MoveGroupCommander("arm", '/my_gen3/robot_description', '/my_gen3')
        self.pub_arm = rospy.Publisher(
            name = '/my_gen3/in/joint_velocity', 
            data_class= msg.Base_JointSpeeds, 
            queue_size= 10
        )
        self.joint_speeds = msg.Base_JointSpeeds()
        self.joint_speeds.duration = 0
        #-------------------------------------------------------------------------------------------------
        
        self.jacobian_matrix = None
        self.custom_jacob_sub = rospy.Subscriber('/jacobian_topic', Jacobian, self.custom_jacob_callback)

        while self.jacobian_matrix is None:
            rospy.sleep(0.1)
        

        rospy.Subscriber("keyboard_commands", String, self.keyboard_callback)
        self.twist_c_list = [0, 0, 0, 0, 0, 0]



        pass

    def custom_jacob_callback(self,msg:Jacobian):
        
        jacobian_matrix = np.array(msg.data).reshape((msg.rows,msg.columns)) 
        self.jacobian_matrix = jacobian_matrix
        return(jacobian_matrix)


    def get_jacobian(self) -> np.ndarray:
        state = self.robot.get_current_state().joint_state.position
        state = list(state)
        return np.array(self.group.get_jacobian_matrix(state))   
         
    
    def keyboard_callback(self,data):
        key = data.data
        if key == '':
            
            self.zero_vel()

        if key == 'u':
            self.twist_c_list = [0, 0, 0, 0, 0, 1]
            self.movement()
        
        if key == 'h':

            self.twist_c_list = [0, 0, 0, 0, 0, -1]
            self.movement()
        
        if key == 'i':
            self.twist_c_list = [0, 0, 0, 0, 1, 0]
            self.movement()
        
        if key == 'k':
            self.twist_c_list = [0, 0, 0, 0, -1, 0]
            self.movement()

        if key == 'j':
            self.twist_c_list = [0, 0, 0, 1, 0, 0]
            self.movement()
        
        if key == "l":
            self.twist_c_list = [0, 0, 0, -1, 0, 0]
            self.movement()
        ##-----------------------------------------------------------##
        
        if key == 'r':
            self.twist_c_list = [0, 0, 1, 0, 0, 0]
            self.movement()
        
        if key == "f":
            self.twist_c_list = [0, 0, -1, 0, 0, 0]
            self.movement()

        if key == 'w':
            self.twist_c_list = [0, 1, 0, 0, 0, 0]
            self.movement()
        
        if key == "s":
            self.twist_c_list = [0, -1, 0, 0, 0, 0]
            self.movement()
        
        if key == 'a':
            self.twist_c_list = [1, 0, 0, 0, 0, 0]
            self.movement()
        
        if key == "d":
            self.twist_c_list = [-1, 0, 0, 0, 0, 0]
            self.movement()

    def get_joint_vels(self,twist) -> list:
        
        # twist[twist>2] = 2  ## uncommented
        # twist[twist<-2] = -2  ## uncommented
 
        speeds = (np.linalg.pinv(self.get_jacobian()) @ np.array(twist, dtype=np.float32)).tolist()
        # speeds = (np.linalg.pinv(self.jacobian_matrix) @ np.array(twist, dtype=np.float32)).tolist()
        
        speeds_ar = np.array(speeds)

        speeds_ar[np.abs(speeds_ar)<0.5] = 0  ## needs to be uncommented 
        # speeds_ar[speeds_ar>2] = 2
        # speeds_ar[speeds_ar<-2] = -2
        print('minimium values of speed included ...............\n',speeds_ar,'\n')
        speeds = speeds_ar
        
        speeds = [msg.JointSpeed(joint_identifier=i, value=x[0]) for i, x in enumerate(speeds)]
        return speeds
    
    def end_effector_twist(self,twist_c):
        Q_ef_b,t_ef_b = self.camera_ef_tf('end_effector_link','base_link')
        zero = np.zeros((3,3))
        matrix = np.block([[Q_ef_b, zero],
                           [zero, Q_ef_b]])
        print('this is the transformation \n-------------\n',Q_ef_b)
        # np.block([[Q_ef_b, zero],[zero, Q_ef_b ]])
        # twist_ef_ef = np.array(self.twist_c_list).reshape((6,1))
        twist_ef_ef = twist_c
        
        twist_ef_base = matrix @ twist_ef_ef

        return(twist_ef_base)



    def camera_ef_tf(self,from_frame='camera_link',to_frame='end_effector_link'): ## gives rotation and translation from camera frame to end effector frame
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # Fetching the transform between 'camera_link' and 'end_effector_link'
        transform = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0),rospy.Duration(0.05))
        
        # transform = TransformStamped()
    
        # print(transform.transform.translation)
        trans = [transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z]
        quat_list = [transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w]
        
        rot_matrix = tft.quaternion_matrix(quat_list)[:3,:3]

        # print(rot_matrix,trans)
        return(rot_matrix,trans)
        pass;

    def twist_tf_camera_ef(self,twist_c): ## converts camera twist in conv camera frame of reference to twist of end effector in base frame of reference

        rot_matrix,trans = self.camera_ef_tf('camera_link','end_effector_link')  ## amendment this always remains constant so pass it as an argument
        Q_convcam_cam = np.array([[0, 0, 1],
                                [1, 0, 0],
                                [0, 1, 0]])
        # print(rot_matrix,type(rot_matrix),rot_matrix.shape)
        rot_matrix = rot_matrix @ Q_convcam_cam
        x,y,z = trans
        skew_trans = np.array([[0, -z, y],
                                [z, 0, -x],
                                [-y, x, 0]],dtype=np.float32)
        
        # rot_matrix = np.array(rot_matrix,dtype=np.float32)
        # print('this is the rot_matrix\n',rot_matrix,'\n')
        # print(skew_trans)
        s_r = skew_trans @ rot_matrix
        zero = np.zeros((3,3),dtype=np.float32)
        matrix = np.block([[rot_matrix, s_r],[zero, rot_matrix]])
        twist_ef_ef = matrix @ twist_c
        
        Q_ef_b,t_Ef_b = self.camera_ef_tf('end_effector_link','base_link')

        matrix_base = np.block([[Q_ef_b, zero],[zero, Q_ef_b ]])
        twist_ef_base = matrix_base @ twist_ef_ef

        return(twist_ef_base)
    
    def movement(self):

        
        twist_c = np.array([self.twist_c_list],dtype=np.float32).reshape(-1,1)
        # self.twist_ef = self.twist_tf_camera_ef(twist_c) ## this is for camera frame of reference twist input
        self.twist_ef = self.end_effector_twist(twist_c)
        self.speeds = self.get_joint_vels(self.twist_ef)
        self.joint_speeds.joint_speeds = self.speeds
        self.pub_arm.publish(self.joint_speeds)

    # def x_rot(self,value):
    #     value = 1*value
    #     twist_c = np.array([[0, 0, 0, value, 0, 0]],dtype=np.float32).reshape(-1,1)
    #     self.twist_ef = self.twist_tf_camera_ef(twist_c)
    #     self.speeds = self.get_joint_vels(self.twist_ef)
    #     self.joint_speeds.joint_speeds = self.speeds
    #     self.pub_arm.publish(self.joint_speeds)
    
    # def z_rot(self,value):
    #     value = 1*value
    #     twist_c = np.array([[0, 0, 0, 0, 0, value]],dtype=np.float32).reshape(-1,1)
    #     self.twist_ef = self.twist_tf_camera_ef(twist_c)
    #     self.speeds = self.get_joint_vels(self.twist_ef)
    #     self.joint_speeds.joint_speeds = self.speeds
    #     self.pub_arm.publish(self.joint_speeds)

    #     pass

    # def y_trans():
    #     pass

    # def x_trans():
    #     pass

    def zero_vel(self):
        self.speeds = [msg.JointSpeed(joint_identifier=i+1, value = 0.0) for i in range(7)]
        self.joint_speeds.joint_speeds = self.speeds
        self.pub_arm.publish(self.joint_speeds)


if __name__ == "__main__":
    vel = velocity_command()
    while not rospy.is_shutdown():
	    rospy.spin()