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

struct Vector2
{
    float x, y;

    Vector2();
    Vector2(const Vector2& other);
    Vector2(float _x, float _y);
    Vector2(const float* values);

    Vector2 operator +(const Vector2& v) const;
    Vector2 operator -(const Vector2& v) const;
    Vector2 operator *(float s) const;

    Vector2& operator =(const Vector2& other);

    inline operator float* () { return &x; }
    inline operator const float* () const { return &x; }

    inline float& operator [](int index) { return *(&x + index); }
    inline float operator [](int index) const { return *(&x + index); }
};

struct Vector3
{
    float x, y, z;

    Vector3();
    Vector3(const Vector3& other);
    Vector3(float _x, float _y, float _z);
    Vector3(const float* values);

    Vector3 Negate() const{
        Vector3 tmp(-x, -y, -z);
        return tmp;
    }

    Vector3 operator *(const Vector3& v) const;
    Vector3 operator +(const Vector3& v) const;
    Vector3 operator -(const Vector3& v) const;
    Vector3 operator *(float s) const;

    Vector3 operator -() const;

    Vector3& operator =(const Vector3& other);
    Vector3& operator +=(const Vector3& other);
    Vector3& operator *=(float s);

    inline operator float* () { return &x; }
    inline operator const float* () const { return &x; }

    inline float& operator [](int index) { return *(&x + index); }
    inline float operator [](int index) const { return *(&x + index); }
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


//=================================functions=======================================
uint8_t FloatToByte(float f);
int32_t ISqrt(int32_t n);
uint32_t NextPow2(uint32_t x);
uint32_t Log2OfPow2(uint32_t x);
uint32_t ReverseBits32(uint32_t bits);
uint32_t Vec3ToUbyte4(const Vector3& v);

float Powof2(float value);

float Vec2Dot(const Vector2& a, const Vector2& b);
float Vec2Length(const Vector2& v);
float Vec2Distance(const Vector2& a, const Vector2& b);

void Vec2Normalize(Vector2& out, const Vector2& v);
void Vec2Subtract(Vector2& out, const Vector2& a, const Vector2& b);

float Vec3Dot(const Vector3& a, const Vector3& b);
float Vec3Length(const Vector3& v);
float Vec3Distance(const Vector3& a, const Vector3& b);


void Vec3Lerp(Vector3& out, const Vector3& a, const Vector3& b, float s);
void Vec3Add(Vector3& out, const Vector3& a, const Vector3& b);
void Vec3Mad(Vector3& out, const Vector3& a, const Vector3& b, float s);
void Vec3Normalize(Vector3& out, const Vector3& v);
void Vec3Scale(Vector3& out, const Vector3& v, float scale);
void Vec3Subtract(Vector3& out, const Vector3& a, const Vector3& b);
void Vec3Cross(Vector3& out, const Vector3& a, const Vector3& b);
