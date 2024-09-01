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

카메라가 오른쪽에 있다는 기준으로 코드를 작성했습니다.


![Screenshot from 2024-07-29 13-55-20](https://github.com/user-attachments/assets/a91bf31b-414f-4efb-8b76-3cfc262e3baf)
![Screenshot from 2024-07-29 13-56-35](https://github.com/user-attachments/assets/0c80f77a-4504-4e3d-8035-70481e8ed4df)
