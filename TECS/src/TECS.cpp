#include "TECS.h"

void TECS::begin(float k1, float KI_pitch, float KD_pitch , float pitch_ff ,float KI_thr , float thr_ff )// default değerler headerda
{
    _k1 = k1;
    _KI_pitch = KI_pitch;
    _KD_pitch = KD_pitch;
    _pitch_ff = pitch_ff;
    _KI_thr = KI_thr;
    _thr_ff = thr_ff;

}

void TECS::update(float h, float h_dot, float v, float v_dot, float h_sp, float h_dot_sp, float v_sp, float v_dot_sp, float dt){ // v_dot da almalı  içine

    _h = h;
    _h_dot = h_dot;
    _v = v;
    _h_sp = h_sp;
    _h_dot_sp = h_dot_sp;
    _v_sp = v_sp;
    _dt = dt;
    _v_dot_sp = v_dot_sp;
    _v_dot = v_dot;

    energy();
    energyBalance();
    balancetoPitch();
    energytoThrottle();

  }

void TECS::energyBalance(){

    B_dot = (_h_dot / _v) - _k1 * (_v_dot / g);

    B_dot_sp = (_h_dot_sp / _v_sp) - _k1 * (_v_dot_sp / g);

    B_dot_error = B_dot_sp - B_dot;

  }

void TECS::energy(){

    E_dot = (_h_dot / _v) + (_v_dot / g);

    E_dot_sp = (_h_dot_sp / _v_sp) + (_v_dot_sp / g);

    E_dot_error = E_dot_sp - E_dot;

  }

void TECS::balancetoPitch(){

    
    pitchTotalError = B_dot_error * _dt + pitchTotalError;
    PitchI = pitchTotalError * _KI_pitch;

    if(PitchI < -1){ pitchTotalError = -1 / (_KI_pitch); } // bunu da kendi içinde limitliyorum
    if(PitchI > 1){ pitchTotalError = 1 / (_KI_pitch); }

    float ff =  _pitch_ff * _h_dot_sp;

    pitchOutput = PitchI + ff; // bu -1, 1 arası outputumuz olacak sonra bunu servo limitlerine göre kullanması kolay olsun diye

    if(pitchOutput < -1){ pitchOutput = -1; }
    if(pitchOutput > 1){ pitchOutput = 1; }

  }

void TECS::energytoThrottle(){

    
    thrTotalError = E_dot_error * _dt + thrTotalError;
    thrI = thrTotalError * _KI_thr;

    if(thrI < -1){ thrTotalError = -1 / (_KI_thr); } // bunu da kendi içinde limitliyorum yüzde yüze limitlendiği için 1 maksimum değer. 0 yüzde 50 thr değerine eşit oluyor
    if(thrI > 1){ thrTotalError = 1 / (_KI_thr); }

    float ff =  _thr_ff * _v_sp; // ff burada bize t_cruise için gerekli değeri sağlamaya yardım edecek

    thrOutput = thrI + ff; // bu -1, 1 arası outputumuz olacak sonra bunu servo limitlerine göre kullanması kolay olsun diye

    if(thrOutput < -1){ thrOutput = -1; }
    if(thrOutput > 1){ thrOutput = 1; }

  }