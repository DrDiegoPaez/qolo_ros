
    ########### Starting ROS Node ###########
    dat_user = Float32MultiArray()
    dat_user.layout.dim.append(MultiArrayDimension())
    dat_user.layout.dim[0].label = 'FSR_read'
    dat_user.layout.dim[0].size = 11
    dat_user.data = [0]*11
    dat_vel = Float32MultiArray()
    dat_vel.layout.dim.append(MultiArrayDimension())
    dat_vel.layout.dim[0].label = 'Velocities: Input - Output'
    dat_vel.layout.dim[0].size = 4
    dat_vel.data = [0]*4

    dat_wheels = Float32MultiArray()
    dat_wheels.layout.dim.append(MultiArrayDimension())
    dat_wheels.layout.dim[0].label = 'Wheels Output'
    dat_wheels.layout.dim[0].size = 2
    dat_wheels.data = [0]*2

    pub_wheels = rospy.Publisher('qolo/wheels', Float32MultiArray, queue_size=1)
    pub_vel = rospy.Publisher('qolo/velocity', Float32MultiArray, queue_size=1)
    pub_emg = rospy.Publisher('qolo/emergency', Bool, queue_size=1)
    pub_user = rospy.Publisher('qolo/user_input', Float32MultiArray, queue_size=1)
    
    pub = rospy.Publisher('qolo', String, queue_size=1)
    rospy.init_node('qolo_control', anonymous=True)
    
    sub = rospy.Subscriber("qolo/remote_commands", Float32MultiArray, callback_remote, queue_size=1)
    rate = rospy.Rate(10) #  20 hz

        # wheels Velocities left and right§
        dat_wheels.data = [Send_DAC0, Send_DAC1]
        # 1 last-cpmmand time stamp
        # 2, 3 commanded V and  W - linear and angular speed 
        # 4, 5 executed V and W --> considers contraints * in simulation could be the same as command 
        dat_vel.data = [last_msg, Command_V, Command_W, Output_V, Output_W]
        # this can be empty no need for simulation 
        dat_user.data = [Xin[0],Xin[1],Xin[2],Xin[3],Xin[4],Xin[5],Xin[6],Xin[7],Xin[8],Xin[9]]
        
        # rospy.loginfo(RosMassage)
        # Emergency flag for when the robot stops
        pub_emg.publish(FlagEmergency)
        pub_vel.publish(dat_vel)
        pub_wheels.publish(dat_wheels)
        pub_user.publish(dat_user)
        # rospy.loginfo(dat_user)
        rospy.loginfo(dat_vel)
        rospy.loginfo(dat_wheels)