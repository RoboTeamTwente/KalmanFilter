#include <stdio.h>
#include <assert.h>
void ArrayMul(int r1,int c1,float a[r1][c1],int r2,int c2,float b[r2][c2],float result[r1][c2]);
void ArrayTranspose(int r1,int c1,float a[r1][c1],float result[c1][r1]);
void ArraySum(int r1,int c1,float a[r1][c1],float b[r1][c1],float result[r1][c1]);
void ArraySubstract(int r1,int c1,float a[r1][c1],float b[r1][c1],float result[r1][c1]);
void Array3Mul(int r1,int c1,float a[r1][c1],int r2,int c2,float b[r2][c2],int r3,int c3,float c[r3][c3],float result[r1][c3]);
void printArray(int r,int c,float a[r][c]);
