#pragma once
#include "Arduino.h"

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

struct Vector4
{
    float x, y, z, w;

    Vector4();
    Vector4(const Vector4& other);
    Vector4(const Vector3& v, float w);
    Vector4(const Vector2& v, float z, float w);
    Vector4(float _x, float _y, float _z, float _w);
    Vector4(const float* values);

    Vector4 operator +(const Vector4& v) const;
    Vector4 operator -(const Vector4& v) const;
    Vector4 operator *(float s) const;
    Vector4 operator /(float s) const;

    Vector4& operator =(const Vector4& other);
    Vector4& operator /=(float s);

    inline operator float* () { return &x; }
    inline operator const float* () const { return &x; }

    inline float& operator [](int index) { return *(&x + index); }
    inline float operator [](int index) const { return *(&x + index); }
};


struct Matrix3x3
{
public:
    float m[3][3];

    Matrix3x3();

    Matrix3x3(const Matrix3x3& a)
    {
        m[0][0] = a.m[0][0]; m[0][1] = a.m[0][1]; m[0][2] = a.m[0][2];
        m[1][0] = a.m[1][0]; m[1][1] = a.m[1][1]; m[1][2] = a.m[1][2];
        m[2][0] = a.m[2][0]; m[2][1] = a.m[2][1]; m[2][2] = a.m[2][2];
    }

    Vector3 operator*(const Vector3& v) const
    {
        Vector3 r;

        r.x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z;
        r.y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z;
        r.z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z;

        return r;
    }

    inline Matrix3x3 operator*(const Matrix3x3& Right) const
    {
        Matrix3x3 Ret;

        for (unsigned int i = 0 ; i < 3 ; i++) {
            for (unsigned int j = 0 ; j < 3 ; j++) {
                Ret.m[i][j] = m[i][0] * Right.m[0][j] +
                              m[i][1] * Right.m[1][j] +
                              m[i][2] * Right.m[2][j];
            }
        }

        return Ret;
    }

    Matrix3x3 Transpose() const
    {
        Matrix3x3 n;

        for (unsigned int i = 0 ; i < 3 ; i++) {
            for (unsigned int j = 0 ; j < 3 ; j++) {
                n.m[i][j] = m[j][i];
            }
        }

        return n;
    }
};

struct Matrix
{
    // NOTE: row-major (multiply with it from the right)

    float _11, _12, _13, _14;
    float _21, _22, _23, _24;
    float _31, _32, _33, _34;
    float _41, _42, _43, _44;	// translation goes here

    float matrix_array[4][4] = {
        {_11, _12, _13, _14},
        {_21, _22, _23, _24},
        {_31, _32, _33, _34},
        {_41, _42, _43, _44}
    };

    Matrix();
    Matrix(const Matrix& other);
    Matrix(float v11, float v22, float v33, float v44);
    Matrix(
        float v11, float v12, float v13, float v14,
        float v21, float v22, float v23, float v24,
        float v31, float v32, float v33, float v34,
        float v41, float v42, float v43, float v44);
    Matrix(const float* values);

    void RotationX(float x)
    {	
        _11 = 1.0f; _12 = 0.0f   ;  _13 = 0.0f    ; _14 = 0.0f;
        _21 = 0.0f; _22 = cosf(x);  _23 = sinf(x);  _24 = 0.0f;
        _31 = 0.0f; _32 = -sinf(x); _33 = cosf(x) ; _34 = 0.0f;
        _41 = 0.0f; _42	= 0.0f   ;  _43 = 0.0f    ; _44 = 1.0f;
    }

    void RotationY(float y)
    {
        _11 = cosf(y); _12 = 0.0f;  _13 = -sinf(y) ; _14 = 0.0f;
        _21 = 0.0f   ; _22 = 1.0f;  _23 = 0.0f     ; _24 = 0.0f;
        _31 = sinf(y); _32 = 0.0f;  _33 = cosf(y)  ; _34 = 0.0f;
        _41 = 0.0f   ; _42	=0.0f;  _43 = 0.0f     ; _44 = 1.0f;
    }
    void RotationZ(float z)
    {
        _11 = cosf(z) ; _12 = sinf(z);  _13 = 0.0f; _14 = 0.0f;
        _21 = -sinf(z); _22 = cosf(z);  _23 = 0.0f;  _24 =0.0f;
        _31 = 0.0f    ; _32 = 0.0f   ;  _33 = 1.0f; _34 = 0.0f;
        _41 = 0.0f    ; _42	= 0.0f   ;  _43 = 0.0f; _44 = 1.0f;
    }
    Matrix& operator =(const Matrix& other);
    Matrix operator *(const Matrix& other);

    inline operator float* () { return &_11; }
    inline operator const float* () const { return &_11; }

    inline float* operator [](int row) { return (&_11 + 4 * row); }
    inline const float* operator [](int row) const { return (&_11 + 4 * row); }
};

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

void Vec3Transform(Vector4& out, const Vector3& v, const Matrix& m);
void Vec3TransformTranspose(Vector4& out, const Matrix& m, const Vector3& v);
void Vec3TransformCoord(Vector3& out, const Vector3& v, const Matrix& m);
void Vec3TransformCoordTranspose(Vector3& out, const Matrix& m, const Vector3& v);
void Vec3TransformNormal(Vector3& out, const Vector3& v, const Matrix& m);
void Vec3TransformNormalTranspose(Vector3& out, const Matrix& m, const Vector3& v);

void Vec4Lerp(Vector4& out, const Vector4& a, const Vector4& b, float s);
void Vec4Add(Vector4& out, const Vector4& a, const Vector4& b);
void Vec4Subtract(Vector4& out, const Vector4& a, const Vector4& b);
void Vec4Scale(Vector4& out, const Vector4& v, float scale);
void Vec4Transform(Vector4& out, const Vector4& v, const Matrix& m);
void Vec4TransformTranspose(Vector4& out, const Matrix& m, const Vector4& v);

void MatrixIdentity(Matrix& out);
void MatrixInverse(Matrix& out, const Matrix& m);
void MatrixMultiply(Matrix& out, const Matrix& a, const Matrix& b);
void MatrixScaling(Matrix& out, float x, float y, float z);
void MatrixTranspose(Matrix& out, Matrix m);
void MatrixTranslation(Matrix& out, float x, float y, float z);
void MatrixTranslation(Matrix& out, Vector3 data);

