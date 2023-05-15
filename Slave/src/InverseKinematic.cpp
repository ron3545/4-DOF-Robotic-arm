#include "InverseKinematic.h"
#include <math.h>

extern const int NJOINTS;

 void InverseKinematics::Move_To_Pos(End_Effector pos)
 {
    double base_angle = atan2(pos.y, pos.x) * (180 / M_PI);
    double Length_of_Arm = sqrt(pow(pos.x,2) + pow(pos.y,2));
    double Hypotheneus = sqrt(pow(Length_of_Arm,2) + pow(pos.z, 2));
    double phi = atan(pos.z / Length_of_Arm) * (180 / M_PI);
    double theta = acos((Hypotheneus / 2)/75) * (180 / M_PI);

    double angle_link_1 = phi + theta;
    double angle_link_2 = phi - theta;

    current_position = pos;

    joint_angles.push_back(base_angle);
    joint_angles.push_back(angle_link_1);
    joint_angles.push_back(angle_link_2);
 }