//
// Created by zjudancer on 19-1-20.
//

#include <iostream>
#include "ForwardKinematics.h"

int main(int argc, char **argv)
{
    std::vector<double> a = {1,2,3,4,5,6};
    dmotion::ForKin f(a,true);

    return 0;
}
