#include "Mathematics.hpp"

void InverseKinematics_4DOF::Calculate_Joint_Angles(const End_Effector& pos, Joint_Angle& angles)
{   
    thetaE = acos((pow(pos.x,2)+pow(pos.z,2)-pow(Length1,2)-pow(Length2,2))/(2*Length1*Length2));   // 일직선 원점
    thetaQ = acos((pow(pos.x,2)+pow(pos.z,2)+pow(Length1,2)-pow(Length2,2))/(2*Length1*(sqrt((pow(pos.x,2)+pow(pos.z,2))))));
    thetaS = atan2(pos.z,pos.x)-thetaQ;
    new_thetaE = M_PI-thetaE;
    new_thetaS = atan2(pos.z,pos.x)+thetaQ;

    angle_E =(unsigned short)((new_thetaE*180)/M_PI);
    angle_S =(unsigned short)((new_thetaS*180)/M_PI);

    angles.theta1 = atan2(pos.y,pos.x) * (180/M_PI);
    angles.theta2 = (angle_S > 120)? -120 : -angle_S;
    angles.theta3 = (-angle_E < 120)? -120 : -angle_E;

    //Serial.println("base: " + String(angles.theta1) + " ; joint2: " + String(angles.theta2) + " ; joint3: " + String(angles.theta3));
}
