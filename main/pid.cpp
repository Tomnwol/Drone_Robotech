#include "NRF_receiver.hpp"
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


Vector3f get_torque(Vector3f omega_set, Vector3f omega_gyro, Vector3f prev_gyro, float dt){

    if(dt < 0.0005f) dt = 0.0005f; // Sécurité
    if(dt > 0.005f) dt = 0.005f;

    #define TORQUE_AMP_MAX 150.0f
    Vector3f err_ang_rate = vec_sub(omega_set, omega_gyro);
    float Kp_ang_rate = QT_KP; //Mettre un Kp plus faible, voir les valeurs min et max de variation, puis remap pour le dshot
    float Ki_ang_rate = QT_KI;
    float Kd_ang_rate = QT_KD;

    /* INTEGRATEUR */
    float integration_max_value = (QT_KI != 0.0f) ? (TORQUE_AMP_MAX / QT_KI) : 0.0f; // Si Ki == 0 => 0
    Vector3f integration_min = {-integration_max_value, -integration_max_value, 0};
    Vector3f integration_max = {integration_max_value, integration_max_value, 0};      // Limite pour l'intégrateur (à ajuster selon essais)
    static Vector3f err_ang_rate_int = {0, 0, 0};
    // Accumulation de l'intégrateur
    err_ang_rate_int = vec_add( err_ang_rate_int , vec_scale(err_ang_rate, dt) );
    // Saturation de l’intégrateur (anti-windup)
    err_ang_rate_int = vec_clamp(err_ang_rate_int, integration_min, integration_max);
    if(joyThrottle < 100){ // On n'intègre pas au sol
      err_ang_rate_int = (Vector3f){0,0,0};
    }

    /* DERIVATEUR */
    Vector3f gyro_derivative = vec_scale(
      vec_sub(omega_gyro, prev_gyro),
      1.0f/dt
    );
    static Vector3f d_filtered = {0,0,0};

    // Filtrage du D -> évite l'échauffement moteur à cause des spikes
    float f_c_d = 70.0f; // ~ 60–90 Hz
    float RC = 1.0f / (2.0f * M_PI * f_c_d);
    float alpha = dt / (RC + dt);

    d_filtered = vec_add(
      d_filtered,
      vec_scale(vec_sub(gyro_derivative, d_filtered), alpha)
    );

    Vector3f P_term = vec_scale(err_ang_rate, Kp_ang_rate);
    Vector3f I_term = vec_scale(err_ang_rate_int, Ki_ang_rate);
    Vector3f D_term = vec_scale(gyro_derivative, -Kd_ang_rate);

    Vector3f torque = vec_add(
      vec_add(
          P_term,
          I_term
      ),
      D_term
    );

    torque = vec_clamp(torque, (Vector3f){-TORQUE_AMP_MAX,-TORQUE_AMP_MAX,0}, (Vector3f){TORQUE_AMP_MAX,TORQUE_AMP_MAX,0});
    return torque;
}
