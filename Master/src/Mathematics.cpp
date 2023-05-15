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

Vector4::Vector4()
{
x = y = z = w = 0;
}

Vector4::Vector4(const Vector4& other)
{
x = other.x;
y = other.y;
z = other.z;
w = other.w;
}

Vector4::Vector4(const Vector3& v, float w)
{
x = v.x;
y = v.y;
z = v.z;

this->w = w;
}

Vector4::Vector4(const Vector2& v, float z, float w)
{
x = v.x;
y = v.y;

this->z = z;
this->w = w;
}

Vector4::Vector4(float _x, float _y, float _z, float _w)
{
x = _x;
y = _y;
z = _z;
w = _w;
}

Vector4::Vector4(const float* values)
{
x = values[0];
y = values[1];
z = values[2];
w = values[3];
}

Vector4 Vector4::operator +(const Vector4& v) const
{
return Vector4(x + v.x, y + v.y, z + v.z, w + v.w);
}

Vector4 Vector4::operator -(const Vector4& v) const
{
return Vector4(x - v.x, y - v.y, z - v.z, w - v.w);
}

Vector4 Vector4::operator *(float s) const
{
return Vector4(x * s, y * s, z * s, w * s);
}

Vector4 Vector4::operator /(float s) const
{
return Vector4(x / s, y / s, z / s, w / s);
}

Vector4& Vector4::operator =(const Vector4& other)
{
x = other.x;
y = other.y;
z = other.z;
w = other.w;

return *this;
}

Vector4& Vector4::operator /=(float s)
{
x /= s;
y /= s;
z /= s;
w /= s;

return *this;
}

Matrix::Matrix()
{
_11 = _12 = _13 = _14 = 0;
_21 = _22 = _23 = _24 = 0;
_31 = _32 = _33 = _34 = 0;
_41 = _42 = _43 = _44 = 0;
}

Matrix::Matrix(const Matrix& other)
{
operator =(other);
}

Matrix::Matrix(float v11, float v22, float v33, float v44)
{
_12 = _13 = _14 = 0;
_21 = _23 = _24 = 0;
_31 = _32 = _34 = 0;
_41 = _42 = _43 = 0;

_11 = v11;
_22 = v22;
_33 = v33;
_44 = v44;
}

Matrix::Matrix(
float v11, float v12, float v13, float v14,
float v21, float v22, float v23, float v24,
float v31, float v32, float v33, float v34,
float v41, float v42, float v43, float v44)
{
_11 = v11;	_21 = v21;	_31 = v31;	_41 = v41;
_12 = v12;	_22 = v22;	_32 = v32;	_42 = v42;
_13 = v13;	_23 = v23;	_33 = v33;	_43 = v43;
_14 = v14;	_24 = v24;	_34 = v34;	_44 = v44;
}

Matrix::Matrix(const float* values)
{
_11 = values[0];	_21 = values[4];
_12 = values[1];	_22 = values[5];
_13 = values[2];	_23 = values[6];
_14 = values[3];	_24 = values[7];

_31 = values[8];	_41 = values[12];
_32 = values[9];	_42 = values[13];
_33 = values[10];	_43 = values[14];
_34 = values[11];	_44 = values[15];
}

Matrix& Matrix::operator =(const Matrix& other)
{
_11 = other._11;	_21 = other._21;
_12 = other._12;	_22 = other._22;
_13 = other._13;	_23 = other._23;
_14 = other._14;	_24 = other._24;

_31 = other._31;	_41 = other._41;
_32 = other._32;	_42 = other._42;
_33 = other._33;	_43 = other._43;
_34 = other._34;	_44 = other._44;

return *this;
}

Matrix Matrix::operator *(const Matrix& other)
{
Matrix tmp;
MatrixMultiply(tmp, *this, other);
return tmp;
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

float PlaneDotCoord(const Vector4& plane, const Vector3& p)
{
return (plane.x * p.x + plane.y * p.y + plane.z * p.z + plane.w);
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


void Vec3Transform(Vector4& out, const Vector3& v, const Matrix& m)
{
Vector4 tmp;

tmp[0] = v[0] * m._11 + v[1] * m._21 + v[2] * m._31 + m._41;
tmp[1] = v[0] * m._12 + v[1] * m._22 + v[2] * m._32 + m._42;
tmp[2] = v[0] * m._13 + v[1] * m._23 + v[2] * m._33 + m._43;
tmp[3] = v[0] * m._14 + v[1] * m._24 + v[2] * m._34 + m._44;

out[0] = tmp[0];
out[1] = tmp[1];
out[2] = tmp[2];
out[3] = tmp[3];
}

void Vec3TransformTranspose(Vector4& out, const Matrix& m, const Vector3& v)
{
Vector4 tmp;

tmp[0] = v[0] * m._11 + v[1] * m._12 + v[2] * m._13 + m._14;
tmp[1] = v[0] * m._21 + v[1] * m._22 + v[2] * m._23 + m._24;
tmp[2] = v[0] * m._31 + v[1] * m._32 + v[2] * m._33 + m._34;
tmp[3] = v[0] * m._41 + v[1] * m._42 + v[2] * m._43 + m._44;

out[0] = tmp[0];
out[1] = tmp[1];
out[2] = tmp[2];
out[3] = tmp[3];
}

void Vec3TransformCoord(Vector3& out, const Vector3& v, const Matrix& m)
{
Vector4 tmp;

tmp[0] = v[0] * m._11 + v[1] * m._21 + v[2] * m._31 + m._41;
tmp[1] = v[0] * m._12 + v[1] * m._22 + v[2] * m._32 + m._42;
tmp[2] = v[0] * m._13 + v[1] * m._23 + v[2] * m._33 + m._43;
tmp[3] = v[0] * m._14 + v[1] * m._24 + v[2] * m._34 + m._44;

out[0] = tmp[0] / tmp[3];
out[1] = tmp[1] / tmp[3];
out[2] = tmp[2] / tmp[3];
}

void Vec3TransformCoordTranspose(Vector3& out, const Matrix& m, const Vector3& v)
{
Vector4 tmp;

tmp[0] = v[0] * m._11 + v[1] * m._12 + v[2] * m._13 + m._14;
tmp[1] = v[0] * m._21 + v[1] * m._22 + v[2] * m._23 + m._24;
tmp[2] = v[0] * m._31 + v[1] * m._32 + v[2] * m._33 + m._34;
tmp[3] = v[0] * m._41 + v[1] * m._42 + v[2] * m._43 + m._44;

out[0] = tmp[0] / tmp[3];
out[1] = tmp[1] / tmp[3];
out[2] = tmp[2] / tmp[3];
}

void Vec3TransformNormal(Vector3& out, const Vector3& v, const Matrix& m)
{
// NOTE: expects inverse transpose
Vector3 tmp;

tmp[0] = v[0] * m._11 + v[1] * m._21 + v[2] * m._31;
tmp[1] = v[0] * m._12 + v[1] * m._22 + v[2] * m._32;
tmp[2] = v[0] * m._13 + v[1] * m._23 + v[2] * m._33;

out[0] = tmp[0];
out[1] = tmp[1];
out[2] = tmp[2];
}

void Vec3TransformNormalTranspose(Vector3& out, const Matrix& m, const Vector3& v)
{
// NOTE: expects inverse
Vector3 tmp;

tmp[0] = v[0] * m._11 + v[1] * m._12 + v[2] * m._13;
tmp[1] = v[0] * m._21 + v[1] * m._22 + v[2] * m._23;
tmp[2] = v[0] * m._31 + v[1] * m._32 + v[2] * m._33;

out[0] = tmp[0];
out[1] = tmp[1];
out[2] = tmp[2];
}

void Vec4Lerp(Vector4& out, const Vector4& a, const Vector4& b, float s)
{
float invs = 1.0f - s;

out[0] = a[0] * invs + b[0] * s;
out[1] = a[1] * invs + b[1] * s;
out[2] = a[2] * invs + b[2] * s;
out[3] = a[3] * invs + b[3] * s;
}

void Vec4Add(Vector4& out, const Vector4& a, const Vector4& b)
{
out[0] = a[0] + b[0];
out[1] = a[1] + b[1];
out[2] = a[2] + b[2];
out[3] = a[3] + b[3];
}

void Vec4Subtract(Vector4& out, const Vector4& a, const Vector4& b)
{
out[0] = a[0] - b[0];
out[1] = a[1] - b[1];
out[2] = a[2] - b[2];
out[3] = a[3] - b[3];
}

void Vec4Scale(Vector4& out, const Vector4& v, float scale)
{
out[0] = v[0] * scale;
out[1] = v[1] * scale;
out[2] = v[2] * scale;
out[3] = v[3] * scale;
}

void Vec4Transform(Vector4& out, const Vector4& v, const Matrix& m)
{
float tmp[4];

tmp[0] = v[0] * m._11 + v[1] * m._21 + v[2] * m._31 + v[3] * m._41;
tmp[1] = v[0] * m._12 + v[1] * m._22 + v[2] * m._32 + v[3] * m._42;
tmp[2] = v[0] * m._13 + v[1] * m._23 + v[2] * m._33 + v[3] * m._43;
tmp[3] = v[0] * m._14 + v[1] * m._24 + v[2] * m._34 + v[3] * m._44;

out[0] = tmp[0];
out[1] = tmp[1];
out[2] = tmp[2];
out[3] = tmp[3];
}

void Vec4TransformTranspose(Vector4& out, const Matrix& m, const Vector4& v)
{
float tmp[4];

tmp[0] = v[0] * m._11 + v[1] * m._12 + v[2] * m._13 + v[3] * m._14;
tmp[1] = v[0] * m._21 + v[1] * m._22 + v[2] * m._23 + v[3] * m._24;
tmp[2] = v[0] * m._31 + v[1] * m._32 + v[2] * m._33 + v[3] * m._34;
tmp[3] = v[0] * m._41 + v[1] * m._42 + v[2] * m._43 + v[3] * m._44;

out[0] = tmp[0];
out[1] = tmp[1];
out[2] = tmp[2];
out[3] = tmp[3];
}


void MatrixIdentity(Matrix& out)
{
out._12 = out._13 = out._14 = 0;
out._21 = out._23 = out._24 = 0;
out._31 = out._32 = out._34 = 0;
out._41 = out._42 = out._43 = 0;

out._11 = out._22 = out._33 = out._44 = 1;
}

void MatrixInverse(Matrix& out, const Matrix& m)
{
    float s[6] = {
        m._11 * m._22 - m._12 * m._21,
        m._11 * m._23 - m._13 * m._21,
        m._11 * m._24 - m._14 * m._21,
        m._12 * m._23 - m._13 * m._22,
        m._12 * m._24 - m._14 * m._22,
        m._13 * m._24 - m._14 * m._23
    };

    float c[6] = {
        m._31 * m._42 - m._32 * m._41,
        m._31 * m._43 - m._33 * m._41,
        m._31 * m._44 - m._34 * m._41,
        m._32 * m._43 - m._33 * m._42,
        m._32 * m._44 - m._34 * m._42,
        m._33 * m._44 - m._34 * m._43
    };

    float det = (s[0] * c[5] - s[1] * c[4] + s[2] * c[3] + s[3] * c[2] - s[4] * c[1] + s[5] * c[0]);

#ifdef _DEBUG
    if (fabs(det) < 1e-9f)
        throw 1;
#endif

    float r = 1.0f / det;

    out._11 = r * (m._22 * c[5] - m._23 * c[4] + m._24 * c[3]);
    out._12 = r * (m._13 * c[4] - m._12 * c[5] - m._14 * c[3]);
    out._13 = r * (m._42 * s[5] - m._43 * s[4] + m._44 * s[3]);
    out._14 = r * (m._33 * s[4] - m._32 * s[5] - m._34 * s[3]);

    out._21 = r * (m._23 * c[2] - m._21 * c[5] - m._24 * c[1]);
    out._22 = r * (m._11 * c[5] - m._13 * c[2] + m._14 * c[1]);
    out._23 = r * (m._43 * s[2] - m._41 * s[5] - m._44 * s[1]);
    out._24 = r * (m._31 * s[5] - m._33 * s[2] + m._34 * s[1]);

    out._31 = r * (m._21 * c[4] - m._22 * c[2] + m._24 * c[0]);
    out._32 = r * (m._12 * c[2] - m._11 * c[4] - m._14 * c[0]);
    out._33 = r * (m._41 * s[4] - m._42 * s[2] + m._44 * s[0]);
    out._34 = r * (m._32 * s[2] - m._31 * s[4] - m._34 * s[0]);

    out._41 = r * (m._22 * c[1] - m._21 * c[3] - m._23 * c[0]);
    out._42 = r * (m._11 * c[3] - m._12 * c[1] + m._13 * c[0]);
    out._43 = r * (m._42 * s[1] - m._41 * s[3] - m._43 * s[0]);
    out._44 = r * (m._31 * s[3] - m._32 * s[1] + m._33 * s[0]);
}

void MatrixMultiply(Matrix& out, const Matrix& a, const Matrix& b)
{
    Matrix tmp;

    tmp._11 = a._11 * b._11 + a._12 * b._21 + a._13 * b._31 + a._14 * b._41;
    tmp._12 = a._11 * b._12 + a._12 * b._22 + a._13 * b._32 + a._14 * b._42;
    tmp._13 = a._11 * b._13 + a._12 * b._23 + a._13 * b._33 + a._14 * b._43;
    tmp._14 = a._11 * b._14 + a._12 * b._24 + a._13 * b._34 + a._14 * b._44;

    tmp._21 = a._21 * b._11 + a._22 * b._21 + a._23 * b._31 + a._24 * b._41;
    tmp._22 = a._21 * b._12 + a._22 * b._22 + a._23 * b._32 + a._24 * b._42;
    tmp._23 = a._21 * b._13 + a._22 * b._23 + a._23 * b._33 + a._24 * b._43;
    tmp._24 = a._21 * b._14 + a._22 * b._24 + a._23 * b._34 + a._24 * b._44;

    tmp._31 = a._31 * b._11 + a._32 * b._21 + a._33 * b._31 + a._34 * b._41;
    tmp._32 = a._31 * b._12 + a._32 * b._22 + a._33 * b._32 + a._34 * b._42;
    tmp._33 = a._31 * b._13 + a._32 * b._23 + a._33 * b._33 + a._34 * b._43;
    tmp._34 = a._31 * b._14 + a._32 * b._24 + a._33 * b._34 + a._34 * b._44;

    tmp._41 = a._41 * b._11 + a._42 * b._21 + a._43 * b._31 + a._44 * b._41;
    tmp._42 = a._41 * b._12 + a._42 * b._22 + a._43 * b._32 + a._44 * b._42;
    tmp._43 = a._41 * b._13 + a._42 * b._23 + a._43 * b._33 + a._44 * b._43;
    tmp._44 = a._41 * b._14 + a._42 * b._24 + a._43 * b._34 + a._44 * b._44;

    out = tmp;
}

void MatrixScaling(Matrix& out, float x, float y, float z)
{
    MatrixIdentity(out);

    out._11 = x;
    out._22 = y;
    out._33 = z;
}

void MatrixTranslation(Matrix& out, float x, float y, float z)
{
    MatrixIdentity(out);

    out._41 = x;
    out._42 = y;
    out._43 = z;
}

void MatrixTranslation(Matrix& out, Vector3 data)
{
    MatrixIdentity(out);

    out._41 = data.x;
    out._42 = data.y;
    out._43 = data.z;
}

void MatrixTranspose(Matrix& out, Matrix m)
{
    out._11 = m._11;
    out._12 = m._21;
    out._13 = m._31;
    out._14 = m._41;

    out._21 = m._12;
    out._22 = m._22;
    out._23 = m._32;
    out._24 = m._42;

    out._31 = m._13;
    out._32 = m._23;
    out._33 = m._33;
    out._34 = m._43;

    out._41 = m._14;
    out._42 = m._24;
    out._43 = m._34;
    out._44 = m._44;
}
