#include <vi_ekf/MyMatMath.h>

// B = A
void copy3(float A[3][3], float B[3][3])
{
  B[0][0] = A[0][0];  B[0][1] = A[0][1];  B[0][2] = A[0][2];
  B[1][0] = A[1][0];  B[1][1] = A[1][1];  B[1][2] = A[1][2];
  B[2][0] = A[2][0];  B[2][1] = A[2][1];  B[2][2] = A[2][2];
}

// B = A'
//__________________________________________
void transpose3(float A[3][3], float B[3][3])
{
  B[0][0] = A[0][0];  B[0][1] = A[1][0];  B[0][2] = A[2][0];
  B[1][0] = A[0][1];  B[1][1] = A[1][1];  B[1][2] = A[2][1];
  B[2][0] = A[0][2];  B[2][1] = A[1][2];  B[2][2] = A[2][2];
}

// A = kA
//__________________________________________
void scale3(float A[3][3], float k)
{
  A[0][0] = k*A[0][0];  A[0][1] = k*A[0][1];  A[0][2] = k*A[0][2];
  A[1][0] = k*A[1][0];  A[1][1] = k*A[1][1];  A[1][2] = k*A[1][2];
  A[2][0] = k*A[2][0];  A[2][1] = k*A[2][1];  A[2][2] = k*A[2][2];
}

// a = ka
//________________________________________
void scale3(float a[3], float k)
{
  a[0] = k*a[0];  a[1] = k*a[1];  a[2] = k*a[2];
}

// B = kA
//__________________________________________
void scale3(float A[3][3], float k, float B[3][3])
{
  B[0][0] = k*A[0][0];  B[0][1] = k*A[0][1];  B[0][2] = k*A[0][2];
  B[1][0] = k*A[1][0];  B[1][1] = k*A[1][1];  B[1][2] = k*A[1][2];
  B[2][0] = k*A[2][0];  B[2][1] = k*A[2][1];  B[2][2] = k*A[2][2];
}

// b = ka
//__________________________________________
void scale3(float a[3], float k, float b[3])
{
  b[0] = k*a[0];  b[1] = k*a[1];  b[2] = k*a[2];
}

// c = a + b
//_______________________________________
void add3(float a[3], float b[3], float c[3])
{
  c[0] = a[0]+b[0];  c[1] = a[1]+b[1];  c[2] = a[2]+b[2];
}

// c = a - b
//_______________________________________
void subtract3(float a[3], float b[3], float c[3])
{
  c[0] = a[0]-b[0];  c[1] = a[1]-b[1];  c[2] = a[2]-b[2];
}

// C = A - B
//_______________________________________
void subtract3(float A[3][3], float B[3][3], float C[3][3])
{
  C[0][0] = A[0][0]-B[0][0];  C[0][1] = A[0][1]-B[0][1];  C[0][2] = A[0][2]-B[0][2];  
  C[1][0] = A[1][0]-B[1][0];  C[1][1] = A[1][1]-B[1][1];  C[1][2] = A[1][2]-B[1][2];  
  C[2][0] = A[2][0]-B[2][0];  C[2][1] = A[2][1]-B[2][1];  C[2][2] = A[2][2]-B[2][2];  
}

// c = A*b
//_________________________________________
void multiply3(float A[3][3], float b[3], float c[3])
{
  c[0] = A[0][0]*b[0] + A[0][1]*b[1] + A[0][2]*b[2];
  c[1] = A[1][0]*b[0] + A[1][1]*b[1] + A[1][2]*b[2];
  c[2] = A[2][0]*b[0] + A[2][1]*b[1] + A[2][2]*b[2];
}

// C = A*B
//_________________________________________
void multiply3(float A[3][3], float B[3][3], float C[3][3])
{
  C[0][0] = A[0][0]*B[0][0] + A[0][1]*B[1][0] + A[0][2]*B[2][0];  C[0][1] = A[0][0]*B[0][1] + A[0][1]*B[1][1] + A[0][2]*B[2][1];  C[0][2] = A[0][0]*B[0][2] + A[0][1]*B[1][2] + A[0][2]*B[2][2];
  C[1][0] = A[1][0]*B[0][0] + A[1][1]*B[1][0] + A[1][2]*B[2][0];  C[1][1] = A[1][0]*B[0][1] + A[1][1]*B[1][1] + A[1][2]*B[2][1];  C[1][2] = A[1][0]*B[0][2] + A[1][1]*B[1][2] + A[1][2]*B[2][2];
  C[2][0] = A[2][0]*B[0][0] + A[2][1]*B[1][0] + A[2][2]*B[2][0];  C[2][1] = A[2][0]*B[0][1] + A[2][1]*B[1][1] + A[2][2]*B[2][1];  C[2][2] = A[2][0]*B[0][2] + A[2][1]*B[1][2] + A[2][2]*B[2][2];
}



// a x b = c
//________________________________________
void cross3(float a[3], float b[3], float* c)
{
  c[0] = (a[1]*b[2]) - (b[1]*a[2]);
  c[1] = (a[2]*b[0]) - (b[2]*a[0]);
  c[2] = (a[0]*b[1]) - (b[0]*a[1]);
}

// ||a||
//________________________________________
float norm3(float a[3])
{
  return(sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]));
}

// b = Av
//________________________________________
void veeSkew3(float A[3][3], float b[3])
{
  b[0] = A[2][1];  b[1] = A[0][2];  b[2] = A[1][0];
}

// Matrix Inversion
// _______________________________
int Invert(float* A, int n)
{
	// A = input matrix AND result matrix
	// n = number of rows = number of columns in A (n x n)
	int pivrow;		// keeps track of current pivot row
	int k,i,j;		// k: overall index along diagonal; i: row index; j: col index
	int pivrows[n]; // keeps track of rows swaps to undo at end
	float tmp;		// used for finding max value and making column swaps

	for (k = 0; k < n; k++)
	{
		// find pivot row, the row with biggest entry in current column
		tmp = 0;
		for (i = k; i < n; i++)
		{
			if (abs(A[i*n+k]) >= tmp)	// 'Avoid using other functions inside abs()?'
			{
				tmp = abs(A[i*n+k]);
				pivrow = i;
			}
		}

		// check for singular matrix
		if (A[pivrow*n+k] == 0.0f)
		{      
			return 0;
		}

		// Execute pivot (row swap) if needed
		if (pivrow != k)
		{
			// swap row k with pivrow
			for (j = 0; j < n; j++)
			{
				tmp = A[k*n+j];
				A[k*n+j] = A[pivrow*n+j];
				A[pivrow*n+j] = tmp;
			}
		}
		pivrows[k] = pivrow;	// record row swap (even if no swap happened)

		tmp = 1.0f/A[k*n+k];	// invert pivot element
		A[k*n+k] = 1.0f;		// This element of input matrix becomes result matrix

		// Perform row reduction (divide every element by pivot)
		for (j = 0; j < n; j++)
		{
			A[k*n+j] = A[k*n+j]*tmp;
		}

		// Now eliminate all other entries in this column
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				tmp = A[i*n+k];
				A[i*n+k] = 0.0f;  // The other place where in matrix becomes result mat
				for (j = 0; j < n; j++)
				{
					A[i*n+j] = A[i*n+j] - A[k*n+j]*tmp;
				}
			}
		}
	}

	// Done, now need to undo pivot row swaps by doing column swaps in reverse order
	for (k = n-1; k >= 0; k--)
	{
		if (pivrows[k] != k)
		{
			for (i = 0; i < n; i++)
			{
				tmp = A[i*n+k];
				A[i*n+k] = A[i*n+pivrows[k]];
				A[i*n+pivrows[k]] = tmp;
			}
		}
	}
	return 1;
}


// eye matrix 10x10
void eye(float mat[10][10], float val)
  {
    for (int i=0; i<10; ++i){
      for (int j=0; j<10; ++j){
        if (i==j)  {mat[i][j] = val;}
        else       {mat[i][j] = 0.0;}
      }
    }
  }
  
// eye matrix 7x7
void eye(float mat[7][7], float val)
  {
    for (int i=0; i<7; ++i){
      for (int j=0; j<7; ++j){
        if (i==j)  {mat[i][j] = val;}
        else       {mat[i][j] = 0.0;}
      }
    }
  }

// eye matrix 3x3
void eye(float mat[3][3], float val)
  {
    for (int i=0; i<3; ++i){
      for (int j=0; j<3; ++j){
        if (i==j)  {mat[i][j] = val;}
        else       {mat[i][j] = 0.0;}
      }
    }
  }
  
// eye matrix 13x13
void eye(float mat[13][13], float val)
  {
    for (int i=0; i<13; ++i){
      for (int j=0; j<13; ++j){
        if (i==j)  {mat[i][j] = val;}
        else       {mat[i][j] = 0.0;}
      }
    }
  }
