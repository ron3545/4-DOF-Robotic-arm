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
    float thetaE, thetaQ, thetaS, new_thetaE, new_thetaS;
    short angle_E, angle_S;

    float Height_Link1, Height_Base; //the sum of these two is equal to a1
public:
     InverseKinematics_4DOF() 
        : Length1(0.0f), Length2(0.0f), Height_Link1(0.0f), Height_Base(0.0f)
    {}
    InverseKinematics_4DOF(float Length_Lnk1, float Length_Lnk2, float Link1_Height, float Base_Height) 
        : Length1(Length_Lnk1), Length2(Length_Lnk2), Height_Link1(Link1_Height), Height_Base(Base_Height), 
        thetaE(0.0f), thetaQ(0.0f), thetaS(0.0f), new_thetaE(0.0f), new_thetaS(0.0f),angle_E(0.0f), angle_S(0.0f)
    {}
    void Calculate_Joint_Angles(const End_Effector& pos, Joint_Angle& angle);
};


