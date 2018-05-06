#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "xsens.h"
#include "vision.h"
#include "Array.h"

#define PI 3.141593
//define the variance
#define var_a 0.01*0.01  //acceleration xsens
#define var_theta 0.01*0.01 //orientation xsens
#define var_xpos 0.005*0.005 //position nois
#define var_xvel 0.2*0.2 //velocity processing noise
#define var_z 0.01*0.01  //camera noise


void EKF(){
  NodeXs *head_xs = (NodeXs*)malloc(sizeof(NodeXs*));
  NodeVis *head_vis = (NodeVis *)malloc(sizeof(NodeVis *));
  readVisionTxt(head_vis);
  readXsensTxt(head_xs);
  float delt = 0.08;
  float F[4][4]={{1,0,delt,0},{0,1,0,delt},{0,0,1,0},{0,0,0,1}};
  float F_tran[4][4]={0};
  ArrayTranspose(4,4,F,F_tran);
  float H[2][4]={{1,0,0,0},{0,1,0,0}};
  float H_tran[4][2]={0};
  ArrayTranspose(2,4,H,H_tran);
  float cn[2][2]={{var_z,0},{0,var_z}};
  float cu[3][3]={{var_a,0,0},{0,var_a,0},{0,0,var_theta}};
  float cw[4][4]={{var_xpos,0,0,0},{0,var_xpos,0,0},{0,0,var_xvel,0},{0,0,0,var_xvel}};
  float cx_pred[4][4]={{1,0,0,0},{0,1,0,0},{0,0,0.1,0},{0,0,0,0.1}};
  float x_pred[4][1]={0};
  // float x_upd[4][1]={0};
  // float cx_upd[4][4]={0};

  FILE *file_x_pred=fopen("x_pred.txt","w");
  if(file_x_pred==NULL){
    printf("error writing data to file");
    exit(1);
  }

  while(head_vis->next!=NULL){
  // for(int i=0;i<2;i++){
      //***********************//
      //    innovation matrix  //
      //***********************//
    float S[2][2]={0};
    float temp1[2][4]={0},temp3[2][2]={0};
    //temp1 = H*cx_pred
    ArrayMul(2,4,H,4,4,cx_pred,temp1);
    //temp3=H*cx_pred*H'
    ArrayMul(2,4,temp1,4,2,H_tran,temp3);
    //S=H*cx_pred*H'+cn
    ArraySum(2,2,temp3,cn,S);
    //***********************//
    //    Kalman gain        //
    //***********************//
    float det = 1/(S[0][0]*S[1][1]-S[0][1]*S[1][0]);
    float S_inv[2][2]={{det*S[1][1],-det*S[0][1]},{-det*S[1][0],det*S[0][0]}};
    float K[4][2]={0};
    float temp4[4][2]={0};
    //temp4 = cx_pred*H'
    ArrayMul(4,4,cx_pred,4,2,H_tran,temp4);
    //K=cx_pred*H'*S^(-1)
    ArrayMul(4,2,temp4,2,2,S_inv,K);

    //***********************//
    //    X(i|i-1)           //
    //***********************//
     float zk[2][1]={{head_vis->vis.x_vis},{head_vis->vis.y_vis}};
    // float zk[2][1]={{0},{0}};
    float x_upd[4][1]={0};
    float temp5[2][1]={0},temp6[2][1]={0},temp7[4][1]={0};
    //temp5 = H*x_pred
    ArrayMul(2,4,H,4,1,x_pred,temp5);
    //temp6 = zk-H*x_pred
    ArraySubstract(2,1,zk,temp5,temp6);
    //temp7 = K*(zk-H*x_pred)
    ArrayMul(4,2,K,2,1,temp6,temp7);
    //x_upd = x_pred+K*(zk-H*x_pred)
    // for(int i=0;i<4;i++){x_upd[i][0]=0;}
    ArraySum(4,1,x_pred,temp7,x_upd);
    //***********************//
    //    Cx(i|i-1)          //
    //***********************//
    float cx_upd[4][4]={0};
    float temp8[4][2]={0},temp9[4][4]={0};
    float K_tran[2][4]={0};
    ArrayTranspose(4,2,K,K_tran);
    //temp8=K*S
    ArrayMul(4,2,K,2,2,S,temp8);
    //temp9=K*S*K'
    ArrayMul(4,2,temp8,2,4,K_tran,temp9);
    ArraySubstract(4,4,cx_pred,temp9,cx_upd);

    //***********************//
    //one step prediction    //
    //***********************//
    float u[3][1]={{0},{0},{0}};
    // float u[3][1]={{head_xs->xs.u_xs},{head_xs->xs.v_xs},{head_xs->xs.a_xs}};
    float s=sin(u[2][0]),c=cos(u[2][0]);
    float gfun[4][1]={{0},{0},{delt*(u[0][0]*c-u[1][0]*s)},{delt*(u[1][0]*c+u[0][0]*s)}};
    float temp10[4][1]={0};
    for(int i=0;i<4;i++){x_pred[i][0]=0;}
    //temp10 = F*x_upd
    ArrayMul(4,4,F,4,1,x_upd,temp10);
    ArraySum(4,1,temp10,gfun,x_pred);
    //***********************//
    //   convar prediction   //
    //***********************//
    float G[4][3]={{0,0,0},{0,0,0},{c,-s,-s*u[0][0]-u[1][0]*c},{s,c,u[0][0]*c-u[1][0]*s}};
    float G_tran[3][4]={0};
    ArrayTranspose(4,3,G,G_tran);
    float temp11[4][4]={0},temp12[4][4]={0};
    //temp11=F*cx_upd

    ArrayMul(4,4,F,4,4,cx_upd,temp11);
    //temp12=F*cx_upd*F_tran
    ArrayMul(4,4,temp11,4,4,F_tran,temp12);
    float temp13[4][3]={0},temp14[4][4]={0};
    //temp13=G*cu
    ArrayMul(4,3,G,3,3,cu,temp13);
    //temp14=G*cu*G'
    ArrayMul(4,3,temp13,3,4,G_tran,temp14);
    float temp15[4][3]={0};
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
        cx_pred[i][j]=temp12[i][j]+temp14[i][j]+cw[i][j];
      }
    }
    // ArraySum(4,4,temp12,temp14,temp15);
    // ArraySum(4,4,temp15,cw,cx_pred);

    fprintf(file_x_pred, "%f %f %f %f\n",x_pred[0][0],x_pred[1][0],x_pred[2][0],x_pred[3][0] );
    // // printf("xs:  %d\n",sizeXs(head_xs));
    popNodeXs(&head_xs);
    popNodeXs(&head_xs);
    popNodeVis(&head_vis);
  }
  fclose(file_x_pred);
}

int main(){
  EKF();
}
