#include<stdlib.h>

float* MatrixMulplication(float *a,float *b,int r1,int c1,int r2,int c2){
  float *d = (float *)malloc(r1*c2*sizeof(float *));
  float a1[r1][c1],b1[r2][c2],c[r1][c2];
  for(int i=0;i<r1;i++){
    for(int j=0;j<c1;j++){
      a1[i][j]=a[i*c1+j];
    }
  }

  for(int i=0;i<r2;i++){
    for(int j=0;j<c2;j++){
      b1[i][j]=b[i*c2+j];
    }
  }

  for(int i=0;i<r1;i++){
    for(int j=0;j<c2;j++){
      c[i][j]=0;
      for(int k=0;k<c1;k++){
        c[i][j]+=a1[i][k]*b1[k][j];
      }
    }
  }

  for(int i=0;i<r1;i++){
    for(int j=0;j<c2;j++){
      d[i*c2+j]=c[i][j];
    }
  }
  return d;
}
