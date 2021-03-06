#include <Eigen/Dense>

#include <iostream>

using namespace std;
using namespace Eigen;

     inline Eigen::Vector3f project3dTo2d(Eigen::Vector3f p3, Eigen::Matrix3f K){
        // 2dpoint = K*3dpoint
        Eigen::Vector3f p2;
        p2 = K*p3;

        return p2;
    }

int main(int argc, char** argv){
    Matrix2f m2;
    m2<<1,2,
        3,4;
    cout<<"m2 = "<<m2<<endl;

    MatrixXf m23(2,3);
    m23<<1,2,3,
         4,5,6;

    MatrixXf m(2,3);
    m = m2*m23;
    cout<<m<<endl;

    Vector3f p3;
    p3 << 1,2,3;
    Matrix3f K;
    K << 1,0,3,
        0,2,4,
        0,0,1;
    Vector3f p2;
    p2 = project3dTo2d(p3, K);
    cout<<"p2 = "<<p2<<endl;
    cout<<"p2 x="<<p2(0)<<"  y="<<p2(1)<<endl;
        
    return true;
}