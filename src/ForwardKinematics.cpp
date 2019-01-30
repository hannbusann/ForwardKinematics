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
            angles_.emplace_back(angles[i] * M_PI / 180.0);
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

        for (int i = 0; i < 7; i++) {
            Eigen::AngleAxisd rotate_yaw(theta[i], Eigen::Vector3d(0, 0, 1));
            Eigen::AngleAxisd rotate_roll(alpha[i], Eigen::Vector3d(1, 0, 0));
            T.rotate(rotate_yaw);
            T.translate(Eigen::Vector3d(0, 0, d[i]));
            T.translate(Eigen::Vector3d(a[i], 0, 0));
            T.rotate(rotate_roll);
        }
        //这两步是用于保证脚末端的坐标系在所有关节角都为0时，和世界坐标系统一
        Eigen::AngleAxisd rotate_pitch90(-M_PI / 2, Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd rotate_yaw90(M_PI / 2, Eigen::Vector3d(0, 0, 1));
        T.rotate(rotate_pitch90);
        T.rotate(rotate_yaw90);

        std::cout << T.matrix() << std::endl;


        if (0 == isRight_) {
            result_vector = Matrix2Pose(T);
        } else {
            result_vector = Matrix2Pose(T);
            result_vector[1] = - result_vector[1];
            result_vector[3] = - result_vector[3];
            result_vector[5] = - result_vector[5];

        }

        dmotion::PrintVector(result_vector);

    }

    ForKinPlus::ForKinPlus(dmotion::ForKin &left, dmotion::ForKin &right) {

    }

    std::vector<double> Matrix2Pose(Eigen::Isometry3d M) {
        std::vector<double> pose;
        pose.emplace_back(M.matrix()(0, 3));
        pose.emplace_back(M.matrix()(1, 3));
        pose.emplace_back(M.matrix()(2, 3));

        pose.emplace_back(dmotion::Rad2Deg(dmotion::Atan(M.matrix()(2, 1), M.matrix()(2, 2))));
        pose.emplace_back(dmotion::Rad2Deg(std::asin(-M.matrix()(2, 0))));
        pose.emplace_back(dmotion::Rad2Deg(dmotion::Atan(M.matrix()(1, 0), M.matrix()(0, 0))));

        return pose;
    }


}