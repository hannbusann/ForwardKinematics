//
// Created by zjudancer on 19-1-19
// E-mail: zjufanwu@zju.edu.cn
//

#include <ForwardKinematics.h>

namespace dmotion {
    const double ForKin::upper_leg_length = 12.0;  //大腿的长度
    const double ForKin::lower_leg_length = 12.0;  //小腿的长度
    const double ForKin::ankle_from_ground = 6.0;  //脚踝距离地面的高度
    const double ForKin::half_hip_width = 4.5;     //两髋关节点距离的一半,相当于髋关节点相对于身体中心原点的y方向坐标
    const double ForKin::hip_x_from_origin = 0;    //髋关节点相对于身体中心原点的x方向坐标
    const double ForKin::hip_z_from_origin = 8.0;  //髋关节点相对于身体中心原点的z
    ForKin::ForKin(const std::vector<double> angles, bool isRight) {
        for (unsigned i = 0; i < angles.size(); i++) {
                angles_.emplace_back(angles[i]*M_PI/180.0);
        }
        isRight_ = isRight;
        double distance_tmp = std::sqrt(half_hip_width * half_hip_width + hip_x_from_origin * hip_x_from_origin);
        double angle_tmp = std::atan(hip_x_from_origin / half_hip_width);
        //机器人左腿非标DH参数表
        alpha = {0, M_PI / 2, M_PI / 2, 0, 0, -M_PI / 2, M_PI / 2};
        a = {distance_tmp, 0, 0, upper_leg_length, lower_leg_length, 0, ankle_from_ground};
        d = {0, -hip_z_from_origin, 0, 0, 0, 0, 0,};
        theta = {M_PI / 2 - angle_tmp,
                 angle_tmp + angles_[0],
                 angles_[1] - M_PI / 2,
                 angles_[2],
                 -angles_[3],
                 angles_[4],
                 -angles_[5]};

        T = Eigen::Isometry3d::Identity();

        for(int i = 0; i< 7; i++)
        {
                Eigen::AngleAxisd rotate_yaw(theta[i],Eigen::Vector3d(0,0,1));
                Eigen::AngleAxisd rotate_roll(alpha[i],Eigen::Vector3d(1,0,0));
                T.rotate(rotate_yaw);
                T.translate(Eigen::Vector3d(0,0,d[i]));
                T.translate(Eigen::Vector3d(a[i],0,0));
                T.rotate(rotate_roll);
        }

        Eigen::AngleAxisd rotate_pitch90(-M_PI/2,Eigen::Vector3d(0,1,0));
        Eigen::AngleAxisd rotate_yaw90(M_PI/2,Eigen::Vector3d(0,0,1));
        T.rotate(rotate_pitch90);
        T.rotate(rotate_yaw90);
//
std::cout << T.matrix() << std::endl;


//    Eigen::AngleAxisd rotate_yaw(1.57,Eigen::Vector3d(0,0,1));
//        Eigen::AngleAxisd rotate_roll(2,Eigen::Vector3d(1,0,0));
//        T.rotate(rotate_yaw);
//        T.translate(Eigen::Vector3d(0,0,0));
//        T.translate(Eigen::Vector3d(4.5,0,0));
//        T.rotate(rotate_roll);
//        std::cout <<a[0] <<std::endl;
//        std::cout <<theta[0] <<std::endl;

        roll_result = dmotion::Rad2Deg(dmotion::Atan(T.matrix()(2,1),T.matrix()(2,2)));
        pitch_result = dmotion::Rad2Deg(std::asin(-T.matrix()(2,0)));
        yaw_result = dmotion::Rad2Deg(dmotion::Atan(T.matrix()(1,0),T.matrix()(0,0)));
        x_result = T.matrix()(0,3);
        y_result = T.matrix()(1,3);
        z_result = T.matrix()(2,3);

        result_vector.emplace_back(x_result);
        result_vector.emplace_back(y_result);
        result_vector.emplace_back(z_result);
        result_vector.emplace_back(roll_result);
        result_vector.emplace_back(pitch_result);
        result_vector.emplace_back(yaw_result);

        dmotion::PrintVector(result_vector);


    }

}