#ifndef MyMatrixMath_h
#define MyMatrixMath_h

#include <math.h> //for sqrt
#include <stdlib.h> //for abs
// B = A
void copy3(float A[3][3], float B[3][3]);

// B = A'
//__________________________________________
void transpose3(float A[3][3], float B[3][3]);

// A = kA, // a = ka, // B = kA, // b = ka
//__________________________________________
void scale3(float A[3][3], float k);
void scale3(float a[3], float k);
void scale3(float A[3][3], float k, float B[3][3]);
void scale3(float a[3], float k, float b[3]);


// c = a + b
//_______________________________________
void add3(float a[3], float b[3], float c[3]);

// c = a - b, // C = A - B
//_______________________________________
void subtract3(float a[3], float b[3], float c[3]);
void subtract3(float a[3][3], float b[3][3], float c[3][3]);

// c = A*b, // C = A*B
//_________________________________________
void multiply3(float A[3][3], float b[3], float c[3]);
void multiply3(float A[3][3], float B[3][3], float C[3][3]);

// a x b = c
//________________________________________
void cross3(float a[3], float b[3], float* c);

// ||a||
//________________________________________
float norm3(float a[3]);

// b = Av
//________________________________________
void veeSkew3(float A[3][3], float b[3]);

// A^-1
//___________________________
int Invert(float* A, int n);

// I
void eye(float mat[10][10], float val);
void eye(float mat[7][7], float val);
void eye(float mat[3][3], float val);
void eye(float mat[13][13], float val);

#endif
