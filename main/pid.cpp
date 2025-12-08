#include "types.hpp"
#include "utils.hpp"
#include "pid.hpp"

Vector3f get_angular_rate_command(Euler att, Euler att_desired){
  // Calcul d’erreur
  float err_roll  = att_desired.roll  - att.roll;
  float err_pitch = att_desired.pitch - att.pitch;
  // ignorer le yaw car pas mesuré correctement
  float err_yaw = 0.0f;

  // PID -> Correction proportionnel simple
  float Kp_att = 4.0f;
  float omega_roll  = Kp_att * err_roll;
  float omega_pitch = Kp_att * err_pitch;

  //vitesse angulaire cible
  return {omega_roll, omega_pitch, 0.0f};
}

Vector3f get_torque(Vector3f omega_set, Vector3f omega_gyro, float dt){
    Vector3f err_ang_rate = vec_sub(omega_set, omega_gyro);
    float Kp_ang_rate = 5; //Mettre un Kp plus faible, voir les valeurs min et max de variation, puis remap pour le dshot
    float Ki_ang_rate = 0.5f;
    Vector3f integration_min = {-0.5, -0.5, 0};  
    Vector3f integration_max = {0.5, 0.5, 0};      // Limite pour l'intégrateur (à ajuster selon essais)
    static Vector3f err_ang_rate_int = {0, 0, 0};
    // Accumulation de l'intégrateur
    err_ang_rate_int = vec_add( err_ang_rate_int , vec_scale(err_ang_rate, dt) );
    // Saturation de l’intégrateur (anti-windup)
    err_ang_rate_int = vec_clamp(err_ang_rate_int, integration_min, integration_max);

    Vector3f torque = vec_add(
        vec_scale(err_ang_rate, Kp_ang_rate),
        vec_scale(err_ang_rate_int, Ki_ang_rate)
    );

    //float torque_amplitude = 50.0; // valeur d'amplitude empirique mesurée avec Kp = 5 et Ki = 0.5
    //torque = vec_scale(torque, 500.0f/torque_amplitude); // On veut un torque entre -500 et 500 
    //torque = vec_clamp(torque, (Vector3f){-500,-500,0}, (Vector3f){500,500,0});

    float torque_amplitude = 50.0; // valeur d'amplitude empirique mesurée avec Kp = 5 et Ki = 0.5
    torque = vec_scale(torque, 100.0f/torque_amplitude); // On veut un torque entre -500 et 500 
    torque = vec_clamp(torque, (Vector3f){-100,-100,0}, (Vector3f){100,100,0});
    return torque;
}
