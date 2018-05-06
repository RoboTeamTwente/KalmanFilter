#include <stdio.h>
void ArrayMul(int r1,int c1,float a[r1][c1],int r2,int c2,float b[r2][c2],float result[r1][c2]);
void ArrayTranspose(int r1,int c1,float a[r1][c1],float result[c1][r1]);
void ArraySum(int r1,int c1,float a[r1][c1],float b[r1][c1],float result[r1][c1]);
void ArraySubstract(int r1,int c1,float a[r1][c1],float b[r1][c1],float result[r1][c1]);

// void printArray(int r,int c,float a[r][c]){
//   for(int i=0;i<r;i++){
//     for(int j=0;j<c;j++){
//       printf("%.6f  ",a[i][j]);
//     }
//     printf("\n");
//   }
//   printf("\n");
// }
