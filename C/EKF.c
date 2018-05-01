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
  //initilize the head of the linkedlist of xsens and vision
  NodeXs *head_xs = (NodeXs*)malloc(sizeof(NodeXs*));
  NodeVis *head_vis = (NodeVis *)malloc(sizeof(NodeVis *));
  readVisionTxt(head_vis);
  readXsensTxt(head_xs);
	//printVis(head_vis);
  //printXs(head_xs);

  int f_xs=100;
  float delay = 0.08;
  float delt_xs = (float)1/f_xs;
  int N_ahead = (int) (delay/delt_xs);

  float cn[2][2]={{var_z,0},{0,var_z}};
  // float cu[3][3]={{var_a,0,0},{0,var_a,0},{0,0,var_theta}};
  // float cw[4][4]={{var_xpos,0,0,0},{0,var_xpos,0,0},{0,0,var_xvel,0},{0,0,0,var_xvel}};
  float H[2][4]={{1,0,0,0},{0,1,0,0}};
  float x_pred[4][1]={0};
  float x_upd[4][1]={0};
  float cx_pred[4][4]={{1,0,0,0},{0,1,0,0},{0,0,0.1,0},{0,0,0,0.1}};
  float cx_upd[4][4]={0};
  float S[2][2]={0};
  float K[4][2]={0};
  float F[4][4]={{1,0,delt_xs,0},{0,1,0,delt_xs},{0,0,1,0},{0,0,0,1}};
  //delete the first N_ahead of vision
  // for(int i=0;i<N_ahead;i++){
  //   popNodeVis(&head_vis);
  // }


  FILE *file_x_pred=fopen("x_pred.txt","w");
  if(file_x_pred==NULL){
    printf("error writing data to file");
    exit(1);
  }

  for(int k_index=0;k_index<sizeVis(head_vis);k_index++){
    // if(sizeVis(head_vis)<8){
    if(1){
      for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
          cx_upd[i][j]=cx_pred[i][j];
        }
        x_upd[i][0]=x_pred[i][0];
      }

    }else{
      //************************//
      // innovation covariance  //
      //************************//

      //calculate H*cx_pred
      float s1[2][4]={0};
      ArrayMul(2,4,H,4,4,cx_pred,s1);
      float H_tran[4][2]={0};
      ArrayTranspose(2,4,H,H_tran);
      //calculate (H*cx_pred)*H'
      ArrayMul(2,4,s1,4,2,H_tran,S);
      // S=H*cx_pred*H'+cn
      ArraySum(2,2,S,cn,S);
      //*********************//
      // Kalman Gain Matrix  //
      //*********************//

      //calculate cx_pred*H'
      float k1[4][2]={0};
      ArrayMul(4,4,cx_pred,4,2,H_tran,k1);
      float index=1/(S[0][0]*S[1][1]-S[0][1]*S[1][0]);
      float s_inv[2][2]={{index*S[1][1],-index*S[0][1]},{-index*S[1][0],index*S[0][0]}};

      //calculate (cx_pred*H')*s^(-1)
      ArrayMul(4,2,k1,2,2,s_inv,K);

      //end Kalman Gain Matrix

      //************************//
      // update state estimate  //
      //************************//
      //can be done for all the linked list data with for loop
      //need to be changed for all the nodes in the list
      float zk[2][1]={{head_vis->vis.x_vis},{head_vis->vis.y_vis}};
      float temp1[2][1]={0},temp2[2][1]={0};
      // H*x_pred
      ArrayMul(2,4,H,4,1,x_pred,temp1);
      //calculate zk-H*x_pred
      ArraySubstract(2,1,zk,temp1,temp2);


      float temp3[4][1]={0};
      //calculate x_upd=x_pred+K*(zk-H*x_pred)
      ArrayMul(4,2,K,2,1,temp2,temp3);
      ArraySum(4,1,x_pred,temp3,x_upd);
      // fprintf(file_x_upd, "%f %f %f %f\n",x_upd[0],x_upd[1],x_upd[2],x_upd[3] );
  //end state estimate

    //*****************************//
    // update covariance estimate  //
    //*****************************//
    //calculate K*S
    float temp4[4][2]={0};
    ArrayMul(4,2,K,2,2,S,temp4);

    //calculate cx_pred-K*S*K'
    float K_tran[2][4]={0};
    ArrayTranspose(4,2,K,K_tran);
    float temp5[4][4]={0};
    ArrayMul(4,2,temp4,2,4,K_tran,temp5);
    ArraySubstract(4,4,cx_pred,temp5,cx_upd);
    }//end if


    //************************//
    // prediction             //
    //************************//
    float u[3][1]={{head_xs->xs.u_xs},{head_xs->xs.v_xs},{head_xs->xs.a_xs}};
    //x_pred =F*x_upd+delt_xs*gfun(uk)//F(4,4),
    float c=cos(u[2][0]),s=sin(u[2][0]);

    float gfun[4][1]={{0},{0},{c*u[0][0]-s*u[1][0]},{c*u[1][0]+s*u[0][0]}};
    for(int i=0;i<4;i++){x_pred[i][0]=0;}
    ArrayMul(4,4,F,4,1,x_upd,x_pred);


   float data[4]={0};
    for(int i=0;i<4;i++){
      for(int j=0;j<1;j++){
        x_pred[i][j]+=delt_xs*gfun[i][j];
      }
      data[i]=x_pred[i][0];
    }


    fprintf(file_x_pred, "%f %f %f %f\n",data[0],data[1],data[2],data[3] );
    popNodeXs(&head_xs);
    popNodeVis(&head_vis);
  }//end for loop

  fclose(file_x_pred);
  // fclose(file_x_upd);
}

int main(){
  EKF();
}
