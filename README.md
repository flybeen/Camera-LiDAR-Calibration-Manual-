ROS1 Noetic

라이다 : OUSTER OS2-128

        self.camera_matrix = np.array([[654.424483, 0.000000, 507.498424],
                                       [0.000000, 652.746467, 301.958835],
                                       [0.000000, 0.000000, 1.000000]])
        self.dist_coeffs = np.array([-0.370183, 0.124689, -0.001381, -0.002182, 0.000000])
수동으로 바꾸어주세요


        self.lidar_sub = rospy.Subscriber('/os_cloud_node/points', PointCloud2, self.lidar_callback)
        self.camera_sub = rospy.Subscriber('/rotated_image', Image, self.camera_callback)
토픽 이름 변경해주세요

기본적으로 tf프레임을 토픽을 만들어서 받는걸로 제작했습니다.

수동으로 하실 분은 perform_calibration,publish_transform 함수 수정해주세요

