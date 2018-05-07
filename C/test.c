#include <stdio.h>
#include <stdlib.h>
#include "Array.h"

int main(){
  float a[3][3]={{1,2,3},{1,3,2},{2,3,1}};
  float b[3][2]={{1.0,21},{1.1,1.2},{0.2,0.3}};
  float c[2][1]={{0.3},{0.2}};
  float result[3][1]={0};
  Array3Mul(3,3,a,3,2,b,2,1,c,result);
  printArray(3,1,result);
}
