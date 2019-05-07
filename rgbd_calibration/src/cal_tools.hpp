#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>


struct MyPointXYZ
{
    float x;
    float y;
    float z;
};

namespace CalTool {


void get_plane(MyPointXYZ p1, MyPointXYZ p2, MyPointXYZ p3, float plane[4])
{
    // aX+bY+cZ+d = 0
    float a = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
    float b = (p3.x - p1.x)*(p2.z - p1.z) - (p2.x - p1.x)*(p3.z - p1.z);
    float c = (p2.x - p1.x)*(p3.y - p1.y) - (p3.x - p1.x)*(p2.y - p1.y);
    float d = 0 - (a*p1.x + b*p1.y + c*p1.z);

    plane[0] = a;
    plane[1] = b;
    plane[2] = c;
    plane[3] = d;
}

//normal vector
std::vector<float> get_Noraml(MyPointXYZ p1, MyPointXYZ p2, MyPointXYZ p3)
{
    float a = (p3.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
    float b = (p3.x - p1.x)*(p2.z - p1.z) - (p2.x - p1.x)*(p3.z - p1.z);
    float c = (p2.x - p1.x)*(p3.y - p1.y) - (p3.x - p1.x)*(p2.y - p1.y);
    std::vector<float> vect3;
    vect3.push_back(a);
    vect3.push_back(b);
    vect3.push_back(c);
    
    return vect3;
}

//unit vector
MyPointXYZ get_unit(MyPointXYZ p)
{
    float mod = sqrt(p.x*p.x +p.y*p.y+p.z*p.z);
    MyPointXYZ p2;
    p2.x = p.x/mod; p2.y = p.y/mod; p2.z = p.z/mod;
    return p2;
}

//dot vector
float vector_dot(MyPointXYZ v1, MyPointXYZ v2)
{
    float result = v1.x*v2.x+v1.y*v2.y+v1.z*v2.z;
    return result;
}

//cross vector
MyPointXYZ vector_cross(MyPointXYZ v1, MyPointXYZ v2)
{
    MyPointXYZ result;
    result.x = v1.y*v2.z - v1.z*v2.y;
    result.y = v1.z*v2.x - v1.x*v2.z;
    result.z = v1.x*v2.y - v1.y*v2.x;
    return result;
}

// Ax+By+Cz+D = 0
MyPointXYZ shadow_point(float A, float B, float C, float D, MyPointXYZ p)
{
    float proportion = -D/(A*A+B*B+C*C);
    MyPointXYZ result;
     result.x = proportion*A+p.x;
     result.y = proportion*B+p.y;
     result.z = proportion*C+p.z;
     return result;
}

//point to plane distance
double dis_pt2plane(MyPointXYZ pt, float a, float b, float c, float d)
{
    return ::fabs(a*pt.x + b*pt.y + c*pt.z + d)/sqrt(a*a+b*b+c*c);
}

//calculate the degree between vector
float cal_degree(MyPointXYZ p1, MyPointXYZ p2)
{
    float cos_the = vector_dot(p1,p2)/  \
            (sqrt(p1.x*p1.x+p2.y*p2.y+p1.z*p1.z)*sqrt(p2.x*p2.x+p2.y*p2.y+p2.z*p2.z));

    return ::acos(cos_the);
}

//calculate z, uisng z = ax+by+c
float cal_z(float x, float y, float a, float b, float c)
{
    return (a*x+b*y+c);
}
// ax+by+cz+d=0
double cal_z(double x, double y, double a, double b, double c, double d)
{
    return -(a*x+b*y+d)/c;
}

void cal_ransac(std::vector<MyPointXYZ> points, float fitplane[4])
{
    float pretotal = 0;
    float sigma = 0.1;

    int iter = 1000;

    srand((unsigned)time(NULL));

    for(int cnt = 0; cnt < iter; cnt++)
    {
        int sample[3] = {0};
        sample[0] = rand()%(points.size());      //rand int from 0~points.size()-1
        while(true)
        {
            sample[1] = rand()%(points.size());
            if(sample[1] != sample[0])
                break;
        }
        while(true)
        {
            sample[2] = rand()%(points.size());
            if(sample[2] == sample[1] || sample[2] == sample[0])
                continue;
            else
                break;
        }

        MyPointXYZ n1 = points.at(sample[0]);
        MyPointXYZ n2 = points.at(sample[1]);
        MyPointXYZ n3 = points.at(sample[2]);
        float plane[4] = {0};
        get_plane(n1, n2, n3, plane);

        float total = 0;
        for(int i = 0; i < points.size(); i++)
        {
            float distance = dis_pt2plane(points.at(i), plane[0], plane[1], plane[2], plane[3]);
            if(distance < sigma)
                total += distance;
        }

        if(total > pretotal)
        {
            pretotal = total;
            for(int k = 0; k < 4; k++)
                fitplane[k] = plane[k];
        }
    }

    std::cout << "RANSAC get plane:  a =  " << fitplane[0] << " , b =  " << fitplane[1] << " , c =  " << fitplane[2]
              << " , d = " << fitplane[3] << std::endl;
}

void least_square3D(std::vector<MyPointXYZ> points, float &a, float &b, float &c)
{
    int size = points.size();
    Eigen::Matrix<float, Eigen::Dynamic, 3> matXY;
    matXY.resize(size, 3);
    for(int i = 0; i < points.size(); i++)
    {
        matXY(i,0) = points.at(i).x;
        matXY(i,1) = points.at(i).y;
        matXY(i,2) = 1;
    }
    Eigen::Matrix<float, Eigen::Dynamic, 1> matZ;
    matZ.resize(size, 1);
    for(int i = 0; i < points.size(); i++)
        matZ(i, 0) = points.at(i).z;

    Eigen::Matrix<float, 3, 1> result;
    result = ((matXY.transpose()*matXY).inverse()) * (matXY.transpose()) * matZ;
    a = result(0); b = result(1); c = result(2);
}

//calculate quaterniaon using the rotation matrix
void cal_quaternion(float theta, std::vector<float> vect_axis, float *quat)
{
    if(vect_axis.size() != 3)
        return;

    Eigen::AngleAxisf rotation_vector (theta, Eigen::Vector3f(vect_axis[0], vect_axis[1], vect_axis[2]));
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
    rotation_matrix = rotation_vector.toRotationMatrix();

    Eigen::Quaternionf q = Eigen::Quaternionf(rotation_vector);
    //std::cout << "quaternion = \n" << q.coeffs() << std::endl;

    quat[0] = q.x(); quat[1] = q.y(); quat[2] = q.z(); quat[3] = q.w();
}

//quat: w,x,y,z
void cal_quat2rotation(std::vector<double> quat, Eigen::Matrix3d &m)
{
    if(quat.size() != 4)
        return;

    Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
    //q.w() = quat[3]; q.x() = quat[0]; q.y() = quat[1]; q.z() = quat[2];
    m = q.toRotationMatrix();
}

void cal_eulerRPY(Eigen::Matrix3d m)
{
    Eigen::Vector3d euler_angles = m.eulerAngles(2,1,0); //Z Y X, yaw pitch roll
    //std::cout<< "yaw pitch roll = " << euler_angles.transpose() << std::endl;
}

void cal_eulerRPY(MyPointXYZ v1, MyPointXYZ v2, MyPointXYZ v3, float RPY[3])
{
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
    rotation_matrix << v1.x, v2.x, v3.x, v1.y, v2.y, v3.y, v1.z, v2.z, v3.z;
    //rotation_matrix = rotation_matrix.inverse();

    Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(2,1,0); //Z Y X, yaw pitch roll
    //std::cout<< "yaw pitch roll = " << euler_angles.transpose() << std::endl;

    RPY[0] = euler_angles(2); RPY[1] = euler_angles(1); RPY[2] = euler_angles(0);
}


double R = 1.0;    // processing noise
double Q = 1.0;   // measure noise
double p_predict = 1.0;

double kalman_filter_z(double raw_val)
{
    static double x_last = 0;
    static double p_z = 1.0;
    p_predict = p_z+Q;
    double kg = p_predict / (p_predict + R);            // kalman gain
    double x = x_last + kg * (raw_val - x_last);

    x_last = raw_val;

    p_z = (1 - kg) * p_predict;

    return x;
}

double kalman_filter_r(double raw_val)
{
    static double x_last = 0;
    static double p_r = 1.0;
    p_predict = p_r+Q;
    double kg = p_predict / (p_predict + R);            // kalman gain
    double x = x_last + kg * (raw_val - x_last);

    x_last = raw_val;

    p_r = (1 - kg) * p_predict;

    return x;
}

double kalman_filter_p(double raw_val)
{
    static double x_last = 0;
    static double p_p;
    p_predict = p_p+Q;
    double kg = p_predict / (p_predict + R);            // kalman gain
    double x = x_last + kg * (raw_val - x_last);

    x_last = raw_val;

    p_p = (1 - kg) * p_predict;

    return x;
}


}   //end namespace
