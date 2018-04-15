#include <stdio.h>

void func(int r1,int c1,float a[r1][c1],int r2,int c2,float b[r2][c2],float result[r1][c2]){
  for(int i=0;i<r1;i++){
    for(int j=0;j<c2;j++){
      for(int k=0;k<c1;k++){
        result[i][j]+=a[i][k]*b[k][j];
      }
    }
  }
}

void reverseMatrix(int r1,int c1,float a[r1][c1],float result[c1][r1]){
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

int main() {
  float a[2][2]={{1,2},{3,2}};
  float b[2][3]={{2,3,1},{1,3,5}};
  float c[2][3]={{10,0,1},{1,0,0}};
  // func(2,2,a,2,3,b,c);
  // float d[3][2]={0};
  // reverseMatrix(2,3,b,d);
  float d[2][3]={0};
  ArraySubstract(2,3,b,c,b);
  for(int i=0;i<2;i++){
    for(int j=0;j<3;j++){
      printf("%f ",b[i][j] );
    }
    printf("\n");
  }
  return 0;
}
