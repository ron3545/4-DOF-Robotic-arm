#include "Mathematics.hpp"

void InverseKinematics_4DOF::Calculate_Joint_Angles(const End_Effector& pos, Joint_Angle& angles)
{   
    angles.theta1  = atan2(pos.x, pos.y) * (180 /M_PI);
    double l = sqrt(pos.x * pos.x + pos.y * pos.y);
    double h = sqrt(l*l + pos.z * pos.z);
    double phi = atan(pos.z / l) * (180 / M_PI);
    double theta = acos((h/2) / Length1) * (180 / M_PI);

    angles.theta2 = phi + theta;
    angles.theta3 = phi - theta;
}

Kinematics::Kinematics(int num_of_joints_) {
    num_of_joints = num_of_joints_;
    num_of_joints_declared = 0;

    mat_utils.zero((float*)initial_end_effector_pose, 4, 4);
    mat_utils.zero((float*)joint_screw_axes, 6, 6);
}

void Kinematics::add_joint_axis(float s1, float s2, float s3, float s4, float s5, float s6) {
    joint_screw_axes[num_of_joints_declared][0] = s1;
    joint_screw_axes[num_of_joints_declared][1] = s2;
    joint_screw_axes[num_of_joints_declared][2] = s3;
    joint_screw_axes[num_of_joints_declared][3] = s4;
    joint_screw_axes[num_of_joints_declared][4] = s5;
    joint_screw_axes[num_of_joints_declared][5] = s6;

    num_of_joints_declared++;
}

void Kinematics::add_initial_end_effector_pose(float m11, float m12, float m13, float m14, float m21, float m22, float m23, float m24, float m31, float m32, float m33, float m34, float m41, float m42, float m43, float m44) {
    initial_end_effector_pose[0][0] = m11;
    initial_end_effector_pose[0][1] = m12;
    initial_end_effector_pose[0][2] = m13;
    initial_end_effector_pose[0][3] = m14;

    initial_end_effector_pose[1][0] = m21;
    initial_end_effector_pose[1][1] = m22;
    initial_end_effector_pose[1][2] = m23;
    initial_end_effector_pose[1][3] = m24;

    initial_end_effector_pose[2][0] = m31;
    initial_end_effector_pose[2][1] = m32;
    initial_end_effector_pose[2][2] = m33;
    initial_end_effector_pose[2][3] = m34;
    
    initial_end_effector_pose[3][0] = m41;
    initial_end_effector_pose[3][1] = m42;
    initial_end_effector_pose[3][2] = m43;
    initial_end_effector_pose[3][3] = m44;
}

void Kinematics::forward(float* joint_angles, float* transform) {
    float vec6[6];
    float se3[4][4];
    float exp6[4][4];
    float result[4][4];
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            transform[4 * i + j] = initial_end_effector_pose[i][j];
        }
    }

    for (int i = num_of_joints - 1; i >= 0; i--) {
        mat_utils.mul_scalar(joint_screw_axes[i], joint_angles[i], 1, 6, vec6);
        mat_utils.vec_to_se3(vec6, (float*)se3);
        mat_utils.exp6((float*)se3, (float*)exp6);
        mat_utils.mul_matrix((float*)exp6, (float*)transform, 4, 4, 4, 4, (float*)result);
        mat_utils.copy_matrix((float*)result, 4, 4, (float*)transform);
    }
}

void Kinematics::inverse(float* transform, float* jac, float* pinv, float* A_t, float* AA_t, float* A_tA, float* initial_joint_angles, float ew, float ev, float max_iterations, float* joint_angles) {
    float Tsb[4][4];
    float Tsb_inv[4][4];
    float Tsb_inv_T[4][4];
    float log6[4][4];
    float vec6[6];
    float adj[6][6];
    float Vs[6];
    float w[3];
    float v[3];
    float pinv_Vs[6];
    bool error;
    int i;

    mat_utils.copy_matrix(initial_joint_angles, 1, num_of_joints, joint_angles);
    forward(joint_angles, (float*)Tsb);
    mat_utils.trn_mat_inverse((float*)Tsb, (float*)Tsb_inv);
    mat_utils.mul_matrix((float*)Tsb_inv, (float*)transform, 4, 4, 4, 4, (float*)Tsb_inv_T);
    mat_utils.log6((float*)Tsb_inv_T, (float*)log6);
    mat_utils.se3_to_vec((float*)log6, vec6);
    mat_utils.adjoint((float*)Tsb, (float*)adj);
    mat_utils.mul_vector((float*)adj, vec6, 6, 6, Vs);

    w[0] = Vs[0];
    w[1] = Vs[1];
    w[2] = Vs[2];

    v[0] = Vs[3];
    v[1] = Vs[4];
    v[2] = Vs[5];

    error = (mat_utils.norm(w) > ew) || (mat_utils.norm(v) > ev);
    i = 0;
    
    while (error && i < max_iterations) {
        Serial.print("loop\t");
        Serial.println(i);
        jacobian(joint_angles, (float*)jac);
        mat_utils.pseudo_inverse((float*)jac, (float*)A_t, (float*)AA_t, (float*)A_tA, 6, num_of_joints, (float*)pinv);
        mat_utils.mul_vector((float*)pinv, Vs, num_of_joints, 6, pinv_Vs);
        mat_utils.add_matrix(joint_angles, pinv_Vs, 1, num_of_joints, joint_angles);

        i = i + 1;

        forward(joint_angles, (float*)Tsb);

        mat_utils.trn_mat_inverse((float*)Tsb, (float*)Tsb_inv);
        mat_utils.mul_matrix((float*)Tsb_inv, (float*)transform, 4, 4, 4, 4, (float*)Tsb_inv_T);
        mat_utils.log6((float*)Tsb_inv_T, (float*)log6);
        mat_utils.se3_to_vec((float*)log6, vec6);
        mat_utils.adjoint((float*)Tsb, (float*)adj);
        mat_utils.mul_vector((float*)adj, vec6, 6, 6, Vs);

        w[0] = Vs[0];
        w[1] = Vs[1];
        w[2] = Vs[2];

        v[0] = Vs[3];
        v[1] = Vs[4];
        v[2] = Vs[5];

        error = (mat_utils.norm(w) > ew) || (mat_utils.norm(v) > ev);
    }
}

void Kinematics::jacobian(float* joint_angles, float* jacobian) {
    float transform[4][4];
    float vec6[6];
    float se3[4][4];
    float exp6[4][4];
    float result[4][4];
    float adj[6][6];
    float jacobian_column[6];

    mat_utils.zero((float*)jacobian, 6, num_of_joints);

    mat_utils.identity((float*)transform, 4);
    
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < num_of_joints; j++) {
            jacobian[num_of_joints * i + j] = joint_screw_axes[j][i];
        }
    }

    for (int i = 1; i < num_of_joints; i++) {
        mat_utils.mul_scalar(joint_screw_axes[i-1], joint_angles[i-1], 1, 6, vec6);
        mat_utils.vec_to_se3(vec6, (float*)se3);
        mat_utils.exp6((float*)se3, (float*)exp6);
        mat_utils.mul_matrix((float*)transform, (float*)exp6, 4, 4, 4, 4, (float*)result);
        mat_utils.copy_matrix((float*)result, 4, 4, (float*)transform);

        mat_utils.adjoint((float*)transform, (float*)adj);
        mat_utils.mul_vector((float*)adj, joint_screw_axes[i], 6, 6, jacobian_column);
        for (int j = 0; j < 6; j++) {
            jacobian[num_of_joints * j + i] = jacobian_column[j];
        }
    }
}

MatrixUtils::MatrixUtils() {
  
}

// General matrix methods
void MatrixUtils::print_matrix(float* mat, int r, int c, String message) { 
    Serial.println(message);
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            Serial.print(mat[c * i + j]);
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
}

void MatrixUtils::copy_matrix(float* mat, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat[c * i + j];
        }
    }
}

void MatrixUtils::identity(float* mat, int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) {
                mat[n * i + j] = 1;
            }
            else {
                mat[n * i + j] = 0;
            }
        }
    }
}

void MatrixUtils::zero(float* mat, int r, int c) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            mat[c * i + j] = 0;
        }
    }
}

void MatrixUtils::transpose(float* mat, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[r * j + i] = mat[c * i + j];
        }
    }
}

float MatrixUtils::trace(float* mat, int r) {
    float sum = 0;
    for (int i = 0; i < r; i++) {
        sum += mat[r * i + i]; 
    }
    return sum;
}

int MatrixUtils::inverse(float* A, int n) {
    int pivrow = 0;
    int k, i, j; 
    int pivrows[6];
    float tmp;

    for (k = 0; k < n; k++) {
        tmp = 0;
        for (i = k; i < n; i++) {
            if (abs(A[i * n + k]) >= tmp) {
                tmp = abs(A[i * n + k]);
                pivrow = i;
            }
        }

        if (A[pivrow * n + k] == 0.0f) {
//            Serial.println("Inversion failed due to singular matrix");
            return 0;
        }

        if (pivrow != k) {
            for (j = 0; j < n; j++) {
                tmp = A[k * n + j];
                A[k * n + j] = A[pivrow * n + j];
                A[pivrow * n + j] = tmp;
            }
        }
        pivrows[k] = pivrow; 

        tmp = 1.0f / A[k * n + k];  
        A[k * n + k] = 1.0f;

        for (j = 0; j < n; j++) {
            A[k * n + j] = A[k * n + j] * tmp;
        }

        for (i = 0; i < n; i++) {
            if (i != k) {
                tmp = A[i * n + k];
                A[i * n + k] = 0.0f;
                for (j = 0; j < n; j++) {
                    A[i * n + j] = A[i * n + j] - A[k * n + j] * tmp;
                }
            }
        }
    }

    for (k = n - 1; k >= 0; k--) {
        if (pivrows[k] != k) {
            for (i = 0; i < n; i++) {
                tmp = A[i * n + k];
                A[i * n + k] = A[i * n + pivrows[k]];
                A[i * n + pivrows[k]] = tmp;
            }
        }
    }
    return 1;
}

void MatrixUtils::pseudo_inverse(float* mat, float* A_t, float* AA_t, float* A_tA, int r, int c, float* result) {
    transpose((float*)mat, r, c, (float*)A_t);
    
    mul_matrix((float*)mat, (float*)A_t, r, c, c, r, (float*)AA_t);
    mul_matrix((float*)A_t, (float*)mat, c, r, r, c, (float*)A_tA);
    
    int AA_t_inv_res = inverse((float*)AA_t, r);
    int A_tA_inv_res = inverse((float*)A_tA, c);

    if (AA_t_inv_res == 1) {
        // A+ = A_t * (A * A_t)^-1
        mul_matrix((float*)A_t, (float*)AA_t, c, r, r, r, (float*)result);
    }
    else {
        mul_matrix((float*)A_tA, (float*)A_t, c, c, c, r, (float*)result);
    }
}

// Transformation matrix methods
void MatrixUtils::get_rot_mat(float* mat, float* rot_mat) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rot_mat[3 * i + j] = mat[4 * i + j];
        }
    }
}

void MatrixUtils::get_pos_vec(float* mat, float* pos_vec) {
    for (int i = 0; i < 3; i++) {
        pos_vec[i] = mat[4 * i + 3];
    }
}

void MatrixUtils::create_trn_mat(float* rot_mat, float* pos_vec, float* trn_mat) {
    zero((float*)trn_mat, 4, 4);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            trn_mat[4 * i + j] = rot_mat[3 * i + j];
        }
        trn_mat[4 * i + 3] = pos_vec[i];
    }
    trn_mat[4 * 3 + 3] = 1;
}

void MatrixUtils::trn_mat_inverse(float* mat, float* result) {
    float rot_mat[3][3];
    float pos_vec[3];
    float rot_mat_t[3][3];
    float pos_vec_result[3];

    get_rot_mat((float*)mat, (float*)rot_mat);
    get_pos_vec((float*)mat, pos_vec);
    transpose((float*)rot_mat, 3, 3, (float*)rot_mat_t);
    mul_vector((float*)rot_mat_t, pos_vec, 3, 3, pos_vec_result);
    pos_vec_result[0] = -pos_vec_result[0];
    pos_vec_result[1] = -pos_vec_result[1];
    pos_vec_result[2] = -pos_vec_result[2];
    create_trn_mat((float*)rot_mat_t, pos_vec_result, (float*)result);
}

void MatrixUtils::adjoint(float* mat, float* result) {
    float rot_mat[3][3];
    float pos_vec[3];
    float so3[3][3];
    float bottom_left[3][3];

    zero((float*)result, 6, 6);
    get_rot_mat((float*)mat, (float*)rot_mat);
    get_pos_vec((float*)mat, pos_vec);
    vec_to_so3(pos_vec, (float*)so3);

    mul_matrix((float*)so3, (float*)rot_mat, 3, 3, 3, 3, (float*)bottom_left);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[6 * i + j] = rot_mat[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[6 * (i + 3) + j] = bottom_left[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[6 * (i + 3) + (j + 3)] = rot_mat[i][j];
        }
    }
}

void MatrixUtils::exp3(float* mat, float* result) {
    float w[3];
    float id[3][3];
    float w_mat[3][3];
    float w_mat_sq[3][3];
    float term2[3][3];
    float term3[3][3];

    identity((float*)id, 3);

    so3_to_vec((float*)mat, w);

    if (abs(norm(w)) < 1e-6) {
        copy_matrix((float*)id, 3, 3, (float*)result);
    }
    else {
        float theta = get_angle(w);

        div_scalar((float*)mat, theta, 3, 3, (float*)w_mat);
        mul_matrix((float*)w_mat, (float*)w_mat, 3, 3, 3, 3, (float*)w_mat_sq);


        mul_scalar((float*)w_mat, sin(theta), 3, 3, (float*)term2);
        mul_scalar((float*)w_mat_sq, 1 - cos(theta), 3, 3, (float*)term3);

        add_matrix((float*)id, (float*)term2, 3, 3, result);
        add_matrix((float*)result, (float*)term3, 3, 3, result);
    }
}

void MatrixUtils::exp6(float* mat, float* result) {
    float rot_mat[3][3];
    float pos_vec[3];
    float w[3];
    float id[3][3];
    float w_mat[3][3];
    float w_mat_sq[3][3];
    float result_rot[3][3];
    float result_pos_term1_a[3][3];
    float result_pos_term1_b[3][3];
    float result_pos_term1_c[3][3];
    float result_pos_term1[3][3];
    float result_pos_term2[3];
    float result_pos[3];

    identity((float*)id, 3);

    get_rot_mat((float*)mat, (float*)rot_mat);
    get_pos_vec((float*)mat, pos_vec);
    so3_to_vec((float*)rot_mat, w);

    if (abs(norm(w)) < 1e-6) {
        create_trn_mat((float*)id, pos_vec, (float*)result);
    }
    else {
        float theta = get_angle(w);

        div_scalar((float*)rot_mat, theta, 3, 3, (float*)w_mat);
        exp3((float*)rot_mat, (float*)result_rot);
        mul_matrix((float*)w_mat, (float*)w_mat, 3, 3, 3, 3, (float*)w_mat_sq);

        mul_scalar((float*)id, theta, 3, 3, (float*)result_pos_term1_a);
        mul_scalar((float*)w_mat, 1 - cos(theta), 3, 3, (float*)result_pos_term1_b);
        mul_scalar((float*)w_mat_sq, theta - sin(theta), 3, 3, (float*)result_pos_term1_c);

        add_matrix((float*)result_pos_term1_a, (float*)result_pos_term1_b, 3, 3, (float*)result_pos_term1);
        add_matrix((float*)result_pos_term1, (float*)result_pos_term1_c, 3, 3, (float*)result_pos_term1);

        div_scalar(pos_vec, theta, 1, 3, result_pos_term2);
        mul_vector((float*)result_pos_term1, result_pos_term2, 3, 3, result_pos);

        create_trn_mat((float*)result_rot, result_pos, (float*)result);
    }
}

void MatrixUtils::log3(float* mat, float* result) {
    float acos_input = (trace((float*)mat, 3) - 1.0) / 2.0;

    if (acos_input >= 1) {
        zero((float*)result, 3, 3);
    }
    else if (acos_input <= -1) {
        float w[3];
        float s;
        float so3[3][3];
        
        if (1 + mat[3 * 2 + 2] > 1e-6) {
            w[0] = mat[3 * 0 + 2];
            w[1] = mat[3 * 1 + 2];
            w[2] = 1 + mat[3 * 2 + 2];
            s = 1.0 / sqrt(2.0 * (1 + mat[3 * 2 + 2]));
            mul_scalar(w, s, 1, 3, w);
        }
        else if (1 + mat[3 * 1 + 1] >= 1e-6) {
            w[0] = mat[3 * 0 + 1];
            w[1] = 1 + mat[3 * 1 + 1];
            w[2] = mat[3 * 2 + 1];
            s = 1.0 / sqrt(2.0 * (1 + mat[3 * 1 + 1]));
            mul_scalar(w, s, 1, 3, w);
        }
        else {
            w[0] = 1 + mat[3 * 0 + 0];
            w[1] = mat[3 * 1 + 0];
            w[2] = mat[3 * 2 + 0];
            s = 1.0 / sqrt(2.0 * (1 + mat[3 * 0 + 0]));
            mul_scalar(w, s, 1, 3, w);
        }
        mul_scalar(w, PI, 1, 3, w);
        vec_to_so3(w, (float*)so3);
        copy_matrix((float*)so3, 3, 3, (float*)result);
    }
    else {
        float mat_t[3][3];
        float term[3][3];
        float theta = acos(acos_input);
        float s = theta / 2.0 / sin(theta);
        transpose((float*)mat, 3, 3, (float*)mat_t);
        sub_matrix((float*)mat, (float*)mat_t, 3, 3, (float*)term);
        mul_scalar((float*)term, s, 3, 3, (float*)result);
    }
}

void MatrixUtils::log6(float* mat, float* result) {
    float rot_mat[3][3];
    float w_mat[3][3];
    bool condition = true;

    get_rot_mat((float*)mat, (float*)rot_mat);
    log3((float*)rot_mat, (float*)w_mat);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (w_mat[i][j] != 0) {
                condition = false;
            }
        }
    }
    if (condition == true) {
        float rot[3][3];
        float vec[3];

        zero((float*)rot, 3, 3);
        vec[0] = mat[4 * 0 + 3];
        vec[1] = mat[4 * 1 + 3];
        vec[2] = mat[4 * 2 + 3];

        create_trn_mat((float*)rot, vec, (float*)result);
        result[4 * 3 + 3] = 0;
    }
    else {
        float theta = acos((trace((float*)rot_mat, 3) - 1.0) / 2.0);
        float w_mat_by_2[3][3];
        float id[3][3];
        float w_mat_sq[3][3];
        float w_mat_sq_by_theta[3][3];
        float s;
        float term1[3][3];
        float term2[3][3];
        float term3[3][3];
        float term4[3];
        float p[3];

        identity((float*)id, 3);
        div_scalar((float*)w_mat, 2.0, 3, 3, (float*)w_mat_by_2);
        mul_matrix((float*)w_mat, (float*)w_mat, 3, 3, 3, 3, (float*)w_mat_sq);
        div_scalar((float*)w_mat_sq, theta, 3, 3, (float*)w_mat_sq_by_theta);
        s = 1.0 / theta - 1.0 / tan(theta / 2.0) / 2.0;
        
        sub_matrix((float*)id, (float*)w_mat_by_2, 3, 3, (float*)term1);
        mul_scalar((float*)w_mat_sq_by_theta, s, 3, 3, (float*)term2);
        add_matrix((float*)term1, (float*)term2, 3, 3, (float*)term3);

        term4[0] = mat[4 * 0 + 3];
        term4[1] = mat[4 * 1 + 3];
        term4[2] = mat[4 * 2 + 3];

        mul_vector((float*)term3, term4, 3, 3, p);

        create_trn_mat((float*)w_mat, p, (float*)result);
        result[4 * 3 + 3] = 0;
    }
}

// Vector methods
float MatrixUtils::norm(float* vec) {
    return sqrt(sq(vec[0]) + sq(vec[1]) + sq(vec[2]));
}

float MatrixUtils::get_angle(float* vec) {
    return norm(vec);
}

// Matrix operators
void MatrixUtils::add_scalar(float* mat, float s, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat[c * i + j] + s;
        }
    }
}

void MatrixUtils::sub_scalar(float* mat, float s, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat[c * i + j] - s;
        }
    }
}

void MatrixUtils::mul_scalar(float* mat, float s, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat[c * i + j] * s;
        }
    }
}

void MatrixUtils::div_scalar(float* mat, float s, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat[c * i + j] / s;
        }
    }
}

void MatrixUtils::add_matrix(float* mat1, float* mat2, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat1[c * i + j] + mat2[c * i + j];
        }
    }
}

void MatrixUtils::sub_matrix(float* mat1, float* mat2, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat1[c * i + j] - mat2[c * i + j];
        }
    }
}

void MatrixUtils::mul_matrix(float* mat1, float* mat2, int r1, int c1, int r2, int c2, float* result) {
    for (int i = 0; i < r1; i++) {
        for(int j = 0; j < c2; j++) {
            result[c2 * i + j] = 0;
            for (int k = 0; k < c1; k++) {
                result[c2 * i + j] = result[c2 * i + j] + mat1[c1 * i + k] * mat2[c2 * k + j];
            }
        }
    }
}

void MatrixUtils::mul_vector(float* mat, float* vec, int r, int c, float* result) {
    for (int i = 0; i < c; i++) {
        result[i] = 0;
    }
    
    for(int i = 0; i < r; i++) {
        for(int j = 0; j < c; j++) {
            result[i] = result[i] + (vec[j] * mat[c * i + j]);
        }
    }
}

// Matrix vector methods
void MatrixUtils::vec_to_so3(float* vec, float* result) {
    result[3 * 0 + 0] = 0;
    result[3 * 0 + 1] = -vec[2];
    result[3 * 0 + 2] = vec[1];

    result[3 * 1 + 0] = vec[2];
    result[3 * 1 + 1] = 0;
    result[3 * 1 + 2] = -vec[0];

    result[3 * 2 + 0] = -vec[1];
    result[3 * 2 + 1] = vec[0];
    result[3 * 2 + 2] = 0;
}

void MatrixUtils::so3_to_vec(float* rot_mat, float* result) {
    result[0] = rot_mat[3 * 2 + 1];
    result[1] = rot_mat[3 * 0 + 2];
    result[2] = rot_mat[3 * 1 + 0];
}

void MatrixUtils::se3_to_vec(float* trn_mat, float* result) {
    result[0] = trn_mat[4 * 2 + 1];
    result[1] = trn_mat[4 * 0 + 2];
    result[2] = trn_mat[4 * 1 + 0];
    result[3] = trn_mat[4 * 0 + 3];
    result[4] = trn_mat[4 * 1 + 3];
    result[5] = trn_mat[4 * 2 + 3];
}

void MatrixUtils::vec_to_se3(float* vec, float* result) {
    float v[3] = {vec[0], vec[1], vec[2]};
    float so3_mat[3][3];

    vec_to_so3(v, (float*)so3_mat);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[4 * i + j] = so3_mat[i][j];
        }
    }

    result[4 * 0 + 3] = vec[3];
    result[4 * 1 + 3] = vec[4];
    result[4 * 2 + 3] = vec[5];

    result[4 * 3 + 0] = 0;
    result[4 * 3 + 1] = 0;
    result[4 * 3 + 2] = 0;
    result[4 * 3 + 3] = 0;
}
