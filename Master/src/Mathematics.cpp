#include "Mathematics.hpp"


Vector2::Vector2()
{
x = y = 0;
}

Vector2::Vector2(const Vector2& other)
{
x = other.x;
y = other.y;
}

Vector2::Vector2(float _x, float _y)
{
x = _x;
y = _y;
}

Vector2::Vector2(const float* values)
{
x = values[0];
y = values[1];
}

Vector2 Vector2::operator +(const Vector2& v) const
{
return Vector2(x + v.x, y + v.y);
}

Vector2 Vector2::operator -(const Vector2& v) const
{
return Vector2(x - v.x, y - v.y);
}

Vector2 Vector2::operator *(float s) const
{
return Vector2(x * s, y * s);
}

Vector2& Vector2::operator =(const Vector2& other)
{
x = other.x;
y = other.y;

return *this;
}

Vector3::Vector3()
{
x = y = z = 0;
}

Vector3::Vector3(const Vector3& other)
{
x = other.x;
y = other.y;
z = other.z;
}

Vector3::Vector3(float _x, float _y, float _z)
{
x = _x;
y = _y;
z = _z;
}

Vector3::Vector3(const float* values)
{
x = values[0];
y = values[1];
z = values[2];
}

Vector3 Vector3::operator *(const Vector3& v) const
{
return Vector3(x * v.x, y * v.y, z * v.z);
}

Vector3 Vector3::operator +(const Vector3& v) const
{
return Vector3(x + v.x, y + v.y, z + v.z);
}

Vector3 Vector3::operator -(const Vector3& v) const
{
return Vector3(x - v.x, y - v.y, z - v.z);
}

Vector3 Vector3::operator *(float s) const
{
return Vector3(x * s, y * s, z * s);
}

Vector3 Vector3::operator -() const
{
return Vector3(-x, -y, -z);
}

Vector3& Vector3::operator =(const Vector3& other)
{
x = other.x;
y = other.y;
z = other.z;

return *this;
}

Vector3& Vector3::operator +=(const Vector3& other)
{
x += other.x;
y += other.y;
z += other.z;

return *this;
}

Vector3& Vector3::operator *=(float s)
{
x *= s;
y *= s;
z *= s;

return *this;
}


uint8_t FloatToByte(float f)
{
int32_t i = (int32_t)f;

if (i < 0)
return 0;
else if (i > 255)
return 255;

return (uint8_t)i;
}

int32_t ISqrt(int32_t n)
{
int32_t b = 0;

while (n >= 0) {
n = n - b;
b = b + 1;
n = n - b;
}

return b - 1;
}

uint32_t NextPow2(uint32_t x)
{
--x;

x |= x >> 1;
x |= x >> 2;
x |= x >> 4;
x |= x >> 8;
x |= x >> 16;

return ++x;
}

uint32_t Log2OfPow2(uint32_t x)
{
uint32_t ret = 0;

while (x >>= 1)
++ret;

return ret;
}

uint32_t ReverseBits32(uint32_t bits)
{
bits = (bits << 16) | (bits >> 16);
bits = ((bits & 0x00ff00ff) << 8) | ((bits & 0xff00ff00) >> 8);
bits = ((bits & 0x0f0f0f0f) << 4) | ((bits & 0xf0f0f0f0) >> 4);
bits = ((bits & 0x33333333) << 2) | ((bits & 0xcccccccc) >> 2);
bits = ((bits & 0x55555555) << 1) | ((bits & 0xaaaaaaaa) >> 1);

return bits;
}

uint32_t Vec3ToUbyte4(const Vector3& v)
{
uint32_t ret = 0;
uint8_t* bytes = (uint8_t*)(&ret);

bytes[0] = FloatToByte((v[0] + 1.0f) * (255.0f / 2.0f) + 0.5f);
bytes[1] = FloatToByte((v[1] + 1.0f) * (255.0f / 2.0f) + 0.5f);
bytes[2] = FloatToByte((v[2] + 1.0f) * (255.0f / 2.0f) + 0.5f);

return ret;
}

float Powof2(float value)
{
    return value * value;
}

float Vec2Dot(const Vector2& a, const Vector2& b)
{
return (a.x * b.x + a.y * b.y);
}

float Vec2Length(const Vector2& v)
{
return sqrtf(v.x * v.x + v.y * v.y);
}

float Vec2Distance(const Vector2& a, const Vector2& b)
{
return Vec2Length(a - b);
}

void Vec2Normalize(Vector2& out, const Vector2& v)
{
float il = 1.0f / sqrtf(v.x * v.x + v.y * v.y);

out[0] = v[0] * il;
out[1] = v[1] * il;
}

void Vec2Subtract(Vector2& out, const Vector2& a, const Vector2& b)
{
out.x = a.x - b.x;
out.y = a.y - b.y;
}

float Vec3Dot(const Vector3& a, const Vector3& b)
{
return (a.x * b.x + a.y * b.y + a.z * b.z);
}

float Vec3Length(const Vector3& v)
{
return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

float Vec3Distance(const Vector3& a, const Vector3& b)
{
return Vec3Length(a - b);
}

void Vec3Lerp(Vector3& out, const Vector3& a, const Vector3& b, float s)
{
float invs = 1.0f - s;

out[0] = a[0] * invs + b[0] * s;
out[1] = a[1] * invs + b[1] * s;
out[2] = a[2] * invs + b[2] * s;
}

void Vec3Add(Vector3& out, const Vector3& a, const Vector3& b)
{
out[0] = a[0] + b[0];
out[1] = a[1] + b[1];
out[2] = a[2] + b[2];
}

void Vec3Mad(Vector3& out, const Vector3& a, const Vector3& b, float s)
{
out[0] = a[0] + b[0] * s;
out[1] = a[1] + b[1] * s;
out[2] = a[2] + b[2] * s;
}

void Vec3Normalize(Vector3& out, const Vector3& v)
{
float il = 1.0f / sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);

out[0] = v[0] * il;
out[1] = v[1] * il;
out[2] = v[2] * il;
}

void Vec3Scale(Vector3& out, const Vector3& v, float scale)
{
out[0] = v[0] * scale;
out[1] = v[1] * scale;
out[2] = v[2] * scale;
}

void Vec3Subtract(Vector3& out, const Vector3& a, const Vector3& b)
{
out[0] = a[0] - b[0];
out[1] = a[1] - b[1];
out[2] = a[2] - b[2];
}

void Vec3Cross(Vector3& out, const Vector3& a, const Vector3& b)
{
out[0] = a[1] * b[2] - a[2] * b[1];
out[1] = a[2] * b[0] - a[0] * b[2];
out[2] = a[0] * b[1] - a[1] * b[0];
}




//========================================class impl====================================================
void InverseKinematics_4DOF::Calculate_Joint_Angles(const End_Effector& pos, Joint_Angle& angles)
{   
    Joint_Angle tmp_angle;
    const float a1 =  Height_Link1 + Height_Base;
    tmp_angle.theta1 = atan2(pos.y, pos.x) * (180 / M_PI);  

    float R1 = sqrt((pos.x * pos.x) + (pos.y * pos.y)); 
    float R2 = pos.z - a1; 
    float phi2 = atan2(R2, R1);

    float R3 = sqrt((R1 * R1) + (R2 * R2));
    float phi1 = acos(((Length2 * Length2) - (Length1 * Length1) - (R3 * R3)) / (-2 * Length1 * R3));
    tmp_angle.theta2 = (phi1 - phi2) * (180 / M_PI); //joint 2 angle

    float phi3 = acos(( (R3 * R3) - (Length1 * Length1) - (Length2 * Length2)) / (-2 * Length1 * Length2)) ; 
    tmp_angle.theta3 = ((180 - phi3)) - 92.39; //joint 3 angle

    angles = tmp_angle;
}
