#include<stdio.h>
#include<assert.h>
#include<stdlib.h>

typedef struct{
  int rows;
  int cols;
  float** data;
}Matrix;

Matrix allo_Matrix(int rows,int cols){
  Matrix m;
  m.rows = rows;
  m.cols = cols;
  m.data = (float**)malloc(sizeof(float*)*m.rows*m.cols);
  for(int i=0;i<m.rows;i++){
    for(int j=0;j<m.cols;j++){
      m.data[i][j]=0.0;
    }
  }
}

void multiplyMatrix(Matrix a,Matrix b,Matrix c){
  assert(a.cols==b.rows);
  assert(a.rows==c.rows);
  assert(b.cols==c.cols);
  for(int i=0;i<a.rows;i++){
    for(int j=0;j<b.cols;j++){
      c.data[i][j]=0.0;
      for(int k=0;k<a.cols;k++){
        c.data[i][j]+=a.data[i][k]*b.data[k][j];
      }
    }
  }
}

// void setMatrix(Matrix m,...){
//   for(int i=0;i<m.rows;i++){
//     for(int j=0;j<m.cols;j++){
//
//     }
//   }
// }

int main(){
  
}
