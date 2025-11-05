#include "types.h"
#include "utils.h"

float passe_bas(float value, float last_filtered_value, float dt, float w0){ //diverge is w0*dt est trop grand
    float s1 = w0*dt*(value-last_filtered_value) + last_filtered_value;
    return(s1);
}

void fill_tab(float * tab, int size_tab){
    int i = 0;
    for (i=0; i < size_tab;i++){
        tab[i] = 0;
    }
}

float mean_tab(float * tab, int size_tab){  // Moyenne quadratique pondérée | à remplacer par un filtre passe bas?
    float tot = 0;
    int coef_sum = 0;
    for(int i = 0; i < size_tab;i++){
        tot +=    (size_tab-i)*(size_tab-i)*tab[i];
        coef_sum+=(size_tab-i)*(size_tab-i);
    }
    float moy = tot / coef_sum;
    return(moy);
}


void fill_queue(float * tab, int size_tab, float valeur){ 
    for (int i = size_tab-1; i>=1;i--){
        tab[i] = tab[i-1];
    }
    tab[0] = valeur;
}

Vector3f up_vector(float q0, float q1, float q2, float q3){
    // axe z du drone dans le monde : rotation du (0,0,1)
    float u_x = 2.0*(q1*q3 - q0*q2);
    float u_y = 2.0*(q2*q3 + q0*q1);
    float u_z = 1 - 2.0*(q1*q1 + q2*q2);
    return (Vector3f){u_x, u_y, u_z};
}

// =======================
// Conjugué
// =======================
Quaternion quat_conjugate(Quaternion q) {
    return (Quaternion){q.w, -q.x, -q.y, -q.z};
}

// =======================
// Multiplication quaternion (q1 * q2)
// =======================
Quaternion quat_multiply(Quaternion q1, Quaternion q2) {
    Quaternion r;
    r.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    r.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    r.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    r.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return r;
}

// =======================
// Erreur quaternion → vecteur 3D (approximation linéaire) (ok pour petites erreurs comme dans le cas d'une boucle fermée)
// =======================
Vector3f quat_to_attitude_error(Quaternion q_err) {
    // pour petites erreurs, vecteur 3D proportionnel à l’angle
    float sign_w = (q_err.w >= 0.0f) ? 1.0f : -1.0f;
    float ex = 2.0f * q_err.x * sign_w;
    float ey = 2.0f * q_err.y * sign_w;
    float ez = 2.0f * q_err.z * sign_w;
    return (Vector3f){ex, ey, ez};
}

Vector3f vec_scale(Vector3f v, float s) {
    Vector3f r = {v.x * s, v.y * s, v.z * s};
    return r;
}

Vector3f vec_sub(Vector3f v1, Vector3f v2) {
    Vector3f r = {v1.x-v2.x, v1.y-v2.y, v1.z-v2.z};
    return r;
}

int int_clamp(int value, int min, int max){
    if (value < min){
        return min;
    }else if(value > max){
        return max;
    }
    return value;
}