#include "Array.h"
void ArrayMul(int r1,int c1,float a[r1][c1],int r2,int c2,float b[r2][c2],float result[r1][c2]){
  if(c1!=r2){printf("error!\n");}
  for(int i=0;i<r1;i++){
    for(int j=0;j<c2;j++){
      for(int k=0;k<c1;k++){
        result[i][j]+=a[i][k]*b[k][j];
      }
    }
  }
}

void ArrayTranspose(int r1,int c1,float a[r1][c1],float result[c1][r1]){
  for(int i=0;i<c1;i++){
    for(int j=0;j<r1;j++){
      result[i][j]=a[j][i];
    }
  }
}

void ArraySum(int r1,int c1,float a[r1][c1],float b[r1][c1],float result[r1][c1]){
  for(int i=0;i<r1;i++){
    for(int j=0;j<c1;j++){
      result[i][j]=a[i][j]+b[i][j];
    }
  }
}

void ArraySubstract(int r1,int c1,float a[r1][c1],float b[r1][c1],float result[r1][c1]){
  for(int i=0;i<r1;i++){
    for(int j=0;j<c1;j++){
      result[i][j]=a[i][j]-b[i][j];
    }
  }
}

void Array3Mul(int r1,int c1,float a[r1][c1],int r2,int c2,float b[r2][c2],int r3,int c3,float c[r3][c3],float result[r1][c3]){
  float result1[r1][c2];
  for(int i=0;i<r1;i++){
    for(int j=0;j<r2;j++){
      result1[i][j]=0.0;
    }
  }
  ArrayMul(r1,c1,a,r2,c2,b,result1);
  ArrayMul(r1,c2,result1,r3,c3,c,result);
}
void printArray(int r,int c,float a[r][c]){
  for(int i=0;i<r;i++){
    for(int j=0;j<c;j++){
      printf("%.6f  ",a[i][j]);
    }
    printf("\n");
  }
  printf("\n");
}
