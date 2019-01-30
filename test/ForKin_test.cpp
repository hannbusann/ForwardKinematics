//
// Created by zjudancer on 19-1-20.
//

#include <iostream>
#include "ForwardKinematics.h"

using namespace std;
int main(int argc, char **argv)
{
    //一条腿上的关节角度
    std::vector<double> a = {1,2,3,4,5,6};
    //左腿测试
    //如果不需要输出，在ForwardKinematics.cpp中注掉即可
    cout << "左腿正运动学测试" << endl;
    dmotion::ForKin left(a,false);

    //右腿测试
    cout << "右腿正运动学测试" << endl;
    dmotion::ForKin right(a,true);

    //两腿测试
    cout << "两腿正运动学测试" << endl;
    dmotion::ForKinPlus twoleg(left.result_vector,right.result_vector);

    //可以得到身体中心相对于支撑脚的x,y,z,r,p,y
    //以及摆动脚相对于支撑脚的x,y,z,r,p,y
    cout << "   身体中心相对于支撑脚的x,y,z,r,p,y" << endl;
    dmotion::PrintVector(twoleg.center2support);
    cout << "   摆动脚相对于支撑脚的x,y,z,r,p,y" << endl;
    dmotion::PrintVector(twoleg.hang2support);
    return 0;
}
