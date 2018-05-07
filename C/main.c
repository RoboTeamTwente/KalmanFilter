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
  float delt = 0.01;
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
    float temp1[2][2]={0};
    //temp1 = H*cx_pred*H'
    Array3Mul(2,4,H,4,4,cx_pred,4,2,H_tran,temp1);
    //S=H*cx_pred*H'+cn
    ArraySum(2,2,temp1,cn,S);
    //***********************//
    //    Kalman gain        //
    //***********************//
    float det = 1/(S[0][0]*S[1][1]-S[0][1]*S[1][0]);
    float S_inv[2][2]={{det*S[1][1],-det*S[0][1]},{-det*S[1][0],det*S[0][0]}};
    float K[4][2]={0};

    //K = cx_pred*H'*S^(-1)
    Array3Mul(4,4,cx_pred,4,2,H_tran,2,2,S_inv,K);
    //***********************//
    //    X(i|i-1)           //
    //***********************//
     float zk[2][1]={{head_vis->vis.x_vis},{head_vis->vis.y_vis}};
    // float zk[2][1]={{0},{0}};
    float x_upd[4][1]={0};
    float temp2[2][1]={0},temp3[2][1]={0},temp4[4][1]={0};
    //temp2 = H*x_pred
    ArrayMul(2,4,H,4,1,x_pred,temp2);
    //temp3 = zk-H*x_pred
    ArraySubstract(2,1,zk,temp2,temp3);
    //temp4 = K*(zk-H*x_pred)
    ArrayMul(4,2,K,2,1,temp3,temp4);
    //x_upd = x_pred+K*(zk-H*x_pred)
    // for(int i=0;i<4;i++){x_upd[i][0]=0;}
    ArraySum(4,1,x_pred,temp4,x_upd);
    //***********************//
    //    Cx(i|i-1)          //
    //***********************//
    float cx_upd[4][4]={0};
    float K_tran[2][4]={0};
    ArrayTranspose(4,2,K,K_tran);
    float temp5[4][4]={0};
    //temp5=K*S*K'
    Array3Mul(4,2,K,2,2,S,2,4,K_tran,temp5);
    ArraySubstract(4,4,cx_pred,temp5,cx_upd);

    //***********************//
    //one step prediction    //
    //***********************//
    // float u[3][1]={{0},{0},{0}};
    float u[3][1]={{head_xs->xs.u_xs},{head_xs->xs.v_xs},{head_xs->xs.a_xs}};
    float s=sin(u[2][0]),c=cos(u[2][0]);
    float gfun[4][1]={{0},{0},{delt*(u[0][0]*c-u[1][0]*s)},{delt*(u[1][0]*c+u[0][0]*s)}};
    float temp6[4][1]={0};
    for(int i=0;i<4;i++){x_pred[i][0]=0;}
    //temp10 = F*x_upd
    ArrayMul(4,4,F,4,1,x_upd,temp6);
    ArraySum(4,1,temp6,gfun,x_pred);
    //***********************//
    //   convar prediction   //
    //***********************//
    float G[4][3]={{0,0,0},{0,0,0},{c,-s,-s*u[0][0]-u[1][0]*c},{s,c,u[0][0]*c-u[1][0]*s}};
    float G_tran[3][4]={0};
    ArrayTranspose(4,3,G,G_tran);

    float temp7[4][4]={0};
    //temp7 = F*cx_upd*F'
    Array3Mul(4,4,F,4,4,cx_upd,4,4,F_tran,temp7);
    float temp8[4][4]={0};
    //temp8=G*cu*G'
    Array3Mul(4,3,G,3,3,cu,3,4,G_tran,temp8);
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
        cx_pred[i][j]=temp7[i][j]+temp8[i][j]+cw[i][j];
      }
    }
    fprintf(file_x_pred, "%f %f %f %f\n",x_pred[0][0],x_pred[1][0],x_pred[2][0],x_pred[3][0] );
    popNodeXs(&head_xs);

    float S1[2][2]={0};
    float temp11[2][2]={0};
    //temp11 = H*cx_pred*H'
    Array3Mul(2,4,H,4,4,cx_pred,4,2,H_tran,temp11);
    //S1=H*cx_pred*H'+cn
    ArraySum(2,2,temp11,cn,S1);

    //****//
    // K  //
    //****//
    float det1 = 1/(S1[0][0]*S1[1][1]-S1[0][1]*S1[1][0]);
    float S1_inv[2][2]={{det1*S1[1][1],-det1*S1[0][1]},{-det1*S1[1][0],det1*S1[0][0]}};
    float K1[4][2]={0};

    //K = cx_pred*H'*S^(-1)
    Array3Mul(4,4,cx_pred,4,2,H_tran,2,2,S1_inv,K1);
    //***********************//
    //    X(i|i-1)           //
    //***********************//
    float temp12[2][1]={0},temp13[2][1]={0},temp14[4][1]={0};
    //temp2 = H*x_pred
    ArrayMul(2,4,H,4,1,x_pred,temp12);
    //temp3 = zk-H*x_pred
    ArraySubstract(2,1,zk,temp12,temp13);
    //temp4 = K*(zk-H*x_pred)
    ArrayMul(4,2,K1,2,1,temp13,temp14);
    //x_upd = x_pred+K*(zk-H*x_pred)
    // for(int i=0;i<4;i++){x_upd[i][0]=0;}
    ArraySum(4,1,x_pred,temp14,x_upd);

    //***********************//
    //    Cx(i|i-1)          //
    //***********************//
    float K1_tran[2][4]={0};
    ArrayTranspose(4,2,K1,K1_tran);
    float temp15[4][4]={0};
    //temp5=K*S*K'
    Array3Mul(4,2,K1,2,2,S1,2,4,K1_tran,temp15);
    ArraySubstract(4,4,cx_pred,temp15,cx_upd);

    //***********************//
    //one step prediction    //
    //***********************//
    // float u[3][1]={{0},{0},{0}};
    float u1[3][1]={{head_xs->xs.u_xs},{head_xs->xs.v_xs},{head_xs->xs.a_xs}};
    float s1=sin(u1[2][0]),c1=cos(u1[2][0]);
    float gfun1[4][1]={{0},{0},{delt*(u1[0][0]*c1-u1[1][0]*s1)},{delt*(u1[1][0]*c1+u1[0][0]*s1)}};
    float temp16[4][1]={0};
    for(int i=0;i<4;i++){x_pred[i][0]=0;}
    //temp10 = F*x_upd
    ArrayMul(4,4,F,4,1,x_upd,temp16);
    ArraySum(4,1,temp16,gfun1,x_pred);

    //***********************//
    //   convar prediction   //
    //***********************//
    float G1[4][3]={{0,0,0},{0,0,0},{c1,-s1,-s1*u1[0][0]-u1[1][0]*c1},{s1,c1,u1[0][0]*c1-u1[1][0]*s1}};
    float G1_tran[3][4]={0};
    ArrayTranspose(4,3,G1,G1_tran);

    float temp17[4][4]={0};
    //temp7 = F*cx_upd*F'
    Array3Mul(4,4,F,4,4,cx_upd,4,4,F_tran,temp17);
    float temp18[4][4]={0};
    //temp8=G*cu*G'
    Array3Mul(4,3,G1,3,3,cu,3,4,G1_tran,temp18);
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
        cx_pred[i][j]=temp17[i][j]+temp18[i][j]+cw[i][j];
      }
    }

    fprintf(file_x_pred, "%f %f %f %f\n",x_pred[0][0],x_pred[1][0],x_pred[2][0],x_pred[3][0] );
    popNodeXs(&head_xs);
    popNodeVis(&head_vis);
  }
  fclose(file_x_pred);
}

int main(){
  EKF();
}
