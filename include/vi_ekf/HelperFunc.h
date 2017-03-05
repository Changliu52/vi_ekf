#ifndef __HelperFunc_H__
#define __HelperFunc_H__

void q_mult(double* q1, double* q2, double* q3);
void QuatRot(double *vec, double *q, double *newVec);
void VPos2SPose(double* translation, double* q, double* qVision, double* pos_w);

#endif