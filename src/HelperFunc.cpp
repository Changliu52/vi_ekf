#include <vi_ekf/HelperFunc.h>

void q_mult(double* q1, double* q2, double* q3) 
{
  q3[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
  q3[2] = q1[0]*q2[2] + q1[2]*q2[0] + q1[3]*q2[1] - q1[1]*q2[3];
  q3[3] = q1[0]*q2[3] + q1[3]*q2[0] + q1[1]*q2[2] - q1[2]*q2[1];
  q3[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
}

void QuatRot(double *vec, double *q, double *newVec) 
{
  double q_conj[4];
  q_conj[0] =  q[0];
  q_conj[1] = -q[1];
  q_conj[2] = -q[2];
  q_conj[3] = -q[3];
             
  double temp[4];
  q_mult(q, vec, temp);
  q_mult(temp, q_conj, newVec);
}


void VPos2SPose(double* translation, double* q, double* qVision, double* pos_w)
{
  double qVisionCon[4] = {qVision[0], -qVision[1], -qVision[2], -qVision[3]};
  double pos_s[4];  QuatRot(translation, qVisionCon, pos_s);
  QuatRot(pos_s, q, pos_w);
}
 