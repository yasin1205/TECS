#ifndef TECS_h
#define TECS_h





class TECS
{
 public:

  void begin(float k1 = 1.0f,   float KI_pitch = 0.01, float KD_pitch = 0.01, float pitch_ff = 0.05,float KI_thr = 0.01, float thr_ff = 0.01);
  void update(float h, float h_dot, float v, float v_dot, float h_sp, float h_dot_sp, float v_sp, float v_dot_sp, float dt);
  void energyBalance();
  void energy();
  void balancetoPitch();
  void energytoThrottle();

  float pitchOutput;
  float thrOutput;


 private:
  float _v;
  float _v_dot;
  float _h;
  float _h_dot;

  float g = 9.81f;

  float _v_sp;
  float _v_dot_sp;
  float _h_sp;
  float _h_dot_sp;

  float _KI_pitch;
  float _KD_pitch;
  float _pitch_ff;
  
  
  float _KI_thr;
  float _thr_ff;
  float _KD_thr;

  float _dt;

  float B;
  float B_dot;
  float B_sp;
  float B_dot_sp;

  float E;
  float E_dot;
  float E_sp;
  float E_dot_sp;

  float B_dot_error;
  float E_dot_error;

  float _k1;

  float pitchTotalError = 0;
  float PitchI;

  float thrTotalError = 0;
  float thrI;
};




#endif