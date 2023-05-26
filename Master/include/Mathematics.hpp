#pragma once
#include "Arduino.h"
#include <math.h>
//=================================structs==========================================
struct Motor_Angles 
{
    float Base_Angle, Joint1_Angle, Joint2_Angle;
};

struct End_Effector
{
    float x, y, z;

    End_Effector& operator = (const End_Effector& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;

        return *this;
    }
};

struct Joint_Angle
{
    float theta1,  //waist or base
          theta2,  //Shoulder or Link2
          theta3;  //Elbow or Link 3
};

//=================================class===========================================
class InverseKinematics_4DOF
{ 
private:
    //      a2        a3
    float Length1, Length2;
    float Height_Link1, Height_Base; //the sum of these two is equal to a1
public:
     InverseKinematics_4DOF() 
        : Length1(0.0f), Length2(0.0f), Height_Link1(0.0f), Height_Base(0.0f)
    {}
    InverseKinematics_4DOF(float Length_Lnk1, float Length_Lnk2, float Link1_Height, float Base_Height) 
        : Length1(Length_Lnk1), Length2(Length_Lnk2), Height_Link1(Link1_Height), Height_Base(Base_Height)
    {}
    void Calculate_Joint_Angles(const End_Effector& pos, Joint_Angle& angle);
};


class MatrixUtils {
    private:

    public:
        MatrixUtils();
        
        // General matrix methods
        void print_matrix(float* mat, int r, int c, String message="");
        void copy_matrix(float* mat, int r, int c, float* result);
        void identity(float* mat, int n);
        void zero(float* mat, int r, int c);
        void transpose(float* mat, int r, int c, float* result);
        float trace(float* mat, int r);
        int inverse(float* A, int n);
        void pseudo_inverse(float* mat, float* A_t, float* AA_t, float* A_tA, int r, int c, float* result);

        // Transformation matrix methods
        void get_rot_mat(float* mat, float* rot_mat);
        void get_pos_vec(float* mat, float* pos_vec);
        void create_trn_mat(float* rot_mat, float* pos_vec, float* trn_mat);
        void trn_mat_inverse(float* mat, float* result);
        void adjoint(float* mat, float* result);
        void exp3(float* mat, float* result);
        void exp6(float* mat, float* result);
        void log3(float* mat, float* result);
        void log6(float* mat, float* result);

        // Vector Methods
        float norm(float* vec);
        float get_angle(float* vec);
        
        // Matrix operators
        void add_scalar(float* mat, float s, int r, int c, float* result);
        void sub_scalar(float* mat, float s, int r, int c, float* result);
        void mul_scalar(float* mat, float s, int r, int c, float* result);
        void div_scalar(float* mat, float s, int r, int c, float* result);
        void add_matrix(float* mat1, float* mat2, int r, int c, float* result);
        void sub_matrix(float* mat1, float* mat2, int r, int c, float* result);
        void mul_matrix(float* mat1, float* mat2, int r1, int c1, int r2, int c2, float* result);
        void mul_vector(float* mat1, float* vec, int r, int c, float* result);

        // Matrix vector methods
        void vec_to_so3(float* vec, float* result);
        void so3_to_vec(float* rot_mat, float* result);
        void vec_to_se3(float* vec, float* result);
        void se3_to_vec(float* trn_mat, float* result);
};


class Kinematics {
    private:
        int num_of_joints;
        int num_of_joints_declared;
        float joint_screw_axes[6][6];
        float initial_end_effector_pose[4][4];
        MatrixUtils mat_utils;

    public:
        Kinematics(int num_of_joints_);

        void add_joint_axis(float s1, float s2, float s3, float s4, float s5, float s6);
        void add_initial_end_effector_pose(float m11, float m12, float m13, float m14, float m21, float m22, float m23, float m24, float m31, float m32, float m33, float m34, float m41, float m42, float m43, float m44);
        
        void forward(float* joint_angles, float* transform);
        void inverse(float* transform, float* jac, float* pinv, float* A_t, float* AA_t, float* A_tA, float* initial_joint_angles, float ew, float ev, float max_iterations, float* joint_angles);
        void jacobian(float* joint_angles, float* jacobian);
};

