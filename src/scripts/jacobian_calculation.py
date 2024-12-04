import rospy
from urdf_parser_py.urdf import URDF
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Union
import tf2_ros
from sensor_msgs.msg import JointState
from kortex_examples.msg import Jacobian

class Jacobian_calc():
    def __init__(self):
        rospy.init_node('jacobian_calc',anonymous=True)
        self.jacob_pub = rospy.Publisher('jacobian_topic',Jacobian,queue_size=10)
        # Get the robot description from the parameter server
        robot_description_param = "/my_gen3/robot_description"  # make sure the namespace is added properly
        robot_description = rospy.get_param(robot_description_param)

        # Parse the URDF
        robot = URDF.from_xml_string(robot_description)

        self.joint_states = None
        self.joint_states_sub = rospy.Subscriber('/my_gen3/joint_states',JointState,self.joint_state_callback)
        
        while self.joint_states is None:
            rospy.sleep(0.1)
        

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        

        
        final_link = 'end_effector_link'
        initial_link = 'base_link'
        self.chain = self.get_chain(robot,final_link,initial_link)

        self.jacob_publisher_fun()


    def joint_state_callback(self,data):
        joint_names = data.name
        joint_values = data.position
        self.joint_states = dict(zip(joint_names,joint_values))

    def get_chain(self,robot,final_link,initial_link):
            current_link = final_link
            chain = []
            while current_link != initial_link:
                for joint in robot.joints:
                        if joint.child == current_link:
                            chain.insert(0,joint)
                            current_link = joint.parent
                            break
            return chain
    def get_transformation(self,chain,joint_values:Union[dict,list,np.ndarray]=None):
        
        if joint_values is None:
            joint_values = self.joint_states

        count = 0
        ## to make sure that fixed joints are not taking the value of the joint_values
        ## a counter needs to be increased only when hinge or prismatic joints
        ## have come in the loop across the chain and the fixed joint transformation 
        ## remains as such specified by the urdf
        
        Transformations_array = []    
        T_0_i = np.eye(4)
        i = 0;  ## just for debugging
        for joint in chain:
            epsilon = None
            T_bi_i = np.eye(4)
            position = joint.origin.xyz[:3]
            rpy = joint.origin.rpy[:3]
            # print('\n this is xyz and rpy \n')
            # print(position,rpy)
            T_bi_i[:3,:3] = R.from_euler('xyz',rpy).as_matrix()
            T_bi_i[:3,3] = position

            # print(joint.origin,joint.type)
            # print(T_bi_i)
            # print(joint.name)
            # print('\n--------------------\n')
            
            axis = np.array(joint.axis)
            

            
            if joint.type in ['revolute','continuous']:
                epsilon = 1
                q_value = joint_values[joint.name] if type(joint_values) == dict else joint_values[count]
                rot = R.from_rotvec(axis*q_value).as_matrix()
                #  print(rot)
                T_bi_i[:3,:3] = T_bi_i[:3,:3] @ rot
                count+=1
            if joint.type == 'prismatic':
                epsilon = 0
                q_value = joint_values[joint.name] if type(joint_values) == dict else joint_values[count]
                T_bi_i[:3,3] += axis*q_value
                count+=1
            

            T_0_i = T_0_i @ T_bi_i
            Transformations_array.append(T_0_i)
            # print('this is the count  ',count)
        print('\ntransformation scratch ............')
        print(Transformations_array[-1])
        self.tf_array = Transformations_array
        return(Transformations_array)

    ### helper functions for findiing jacobian

    def skew_matrix_fun(self,vector:Union[np.ndarray,list]):
        
        if len(vector) == 4:
            x,y,z,_ = vector[:,0]
        else:
            x,y,z = vector[:]
        matrix = np.array([[0, -z, y],
                           [z, 0, -x],
                           [-y, x, 0]])
        return(matrix)

    def a_matrix_fun(self,sk_mat):
        eye = np.eye(3,3)
        zeros = np.zeros((3,3))
        matrix = np.block([[eye, zeros],
                            [-sk_mat, eye]])
        
        return(matrix)

    def get_jacobian(self,chain,tf_array)-> np.ndarray: 
        
        nof_joints = len(chain)
        # print(nof_joints,'\n ----------------------------\n\n')
        a_n_i = np.eye(6,6)
        Jacob = np.empty((6,0))
        for i in range(nof_joints-1,-1,-1):
            tf_0_i = tf_array[i]
            joint = chain[i]
            a_bi_i = joint.origin.xyz[:3]
            a_bi_i.append(1)
            a_bi_i = np.array(a_bi_i).reshape(4,-1)
            
            # print(tf_0_i)
            # print(a_bi_i)
            a_0_bi_i = tf_0_i @ a_bi_i;
            a_skew = self.skew_matrix_fun(a_0_bi_i)
            a_mat = self.a_matrix_fun(a_skew)

            a_n_i = a_mat @ a_n_i

            # print('count is --------',i)
            # print(a_n_i)

            if joint.type in ['revolute','continuous']:
                eps = 1
            elif joint.type == 'prismatic':
                eps = 0
            else:
                # print(joint.type) ## -> fixed 
                continue
            
            p = np.block([[eps*tf_0_i[:3,3].reshape(3,-1)],
                        [(1-eps)*tf_0_i[:3,3].reshape(3,-1)]])

            J_ele = a_n_i @ p;
            Jacob = np.concatenate((Jacob,J_ele),axis = 1)

        # print('this is jacob -------------\n')
        # print(Jacob)
        return(Jacob)
    
    def jacob_publisher_fun(self):
        while not rospy.is_shutdown():
            tf_array = self.get_transformation(self.chain,self.joint_states)
            jacobian = self.get_jacobian(self.chain,tf_array)

            flattened_jacob = list(jacobian.flatten())
            ## this flattens the array row wise 
            ## [[1,2],[3,4]] --> [1,2,3,4] 
            
            
            msg = Jacobian()
            
            msg.rows,msg.columns = jacobian.shape
            msg.data = flattened_jacob
            self.jacob_pub.publish(msg)

            ## this is to verify the transformation matrix given by the tf_buffer with the one found from scratch so commented out
            # # Specify the links
            # source_frame = "base_link"
            # target_frame = "end_effector_link"
        
            # try:
            #     # getting the transform
            #     transform = self.tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0), rospy.Duration(1.0))
                
            #     # getting translation and rotation
            #     translation = transform.transform.translation
            #     rotation = transform.transform.rotation

            #     # conveerting quat to rot matrix
            #     from scipy.spatial.transform import Rotation as R
            #     rot_matrix = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()

            #     
            #     transform_matrix = np.eye(4)
            #     transform_matrix[:3, :3] = rot_matrix
            #     transform_matrix[:3, 3] = [translation.x, translation.y, translation.z]

            #     # print('\ntf2 transformation')
            #     # print(f"Transformation Matrix from {source_frame} to {target_frame}:\n{transform_matrix}")

            # except tf2_ros.LookupException:

            #     print("Transform not found")


if __name__ == "__main__":
    try:
        jacob_class = Jacobian_calc()
    except rospy.ROSInterruptException:
        pass


