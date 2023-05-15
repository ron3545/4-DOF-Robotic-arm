#pragma once
#include <math.h>
#include <stdio.h>

#include "DataStructures.hpp"
#include "Mathematics.hpp" 
#include "Arduino.h"

class InverseKinematics
{
private:
    InverseKinematics();
private:
    End_Effector current_position;       

    Linked_List<double> joint_angles;  //base_angle to angle_link_2
public:
    static InverseKinematics& GetInstance()
    {
        static InverseKinematics instance;
        return instance;
    }
    void Move_To_Pos(End_Effector position);

    Linked_List<double> Get_Joint_Angles() const { return joint_angles; }
};