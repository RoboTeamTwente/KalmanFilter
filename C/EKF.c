#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "xsens.h"
#include "vision.h"


#define PI 3.141593
//define the variance
#define var_a 0.01*0.01  //acceleration xsens
#define var_theta 0.01*0.01 //orientation xsens
#define var_xpos 0.005*0.005 //position nois
#define var_xvel 0.2*0.2 //velocity processing noise
#define var_z 0.01*0.01  //camera noise

void readVisionTxt(NodeVis *head_vis){
  FILE *myfile_vis;
	float x_vis,y_vis,a_vis;
	struct vision vis,vis_init;
	myfile_vis = fopen("visionRelevantData.txt","r");
  fscanf(myfile_vis,"%f%f%f",&x_vis,&y_vis,&a_vis);
  vis_init.x_vis=x_vis;
  vis_init.y_vis=y_vis;
  vis_init.a_vis=a_vis/180*PI;
	for(int i=0;i<100;i++){
		fscanf(myfile_vis,"%f%f%f",&x_vis,&y_vis,&a_vis);
		vis.x_vis=x_vis-vis_init.x_vis;
		vis.y_vis=y_vis-vis_init.y_vis;
		vis.a_vis=a_vis/180*PI-vis_init.a_vis;
		insertNodeVis(head_vis,vis);
	}
  fclose(myfile_vis);
}

void readXsensTxt(NodeXs *head_xs){
  FILE *myfile_xs;
  float u_xs,v_xs,a_xs;
  struct xsens xs;
  myfile_xs = fopen("xsensRelevantData.txt","r");
  for(int i=0;i<200;i++){
    fscanf(myfile_xs,"%f%f%f",&u_xs,&v_xs,&a_xs);
    xs.u_xs=u_xs;
    xs.v_xs=v_xs;
    xs.a_xs=a_xs/180*PI;
    insertNodeXs(head_xs,xs);
  }
  fclose(myfile_xs);
}


void writeXsensData(NodeXs *head_xs){
	FILE *file_xs=fopen("file_xsens.txt","w");
	if(file_xs==NULL){
		printf("error writing data to file");
		exit(1);
	}
	NodeXs *current = head_xs;
	for(int i=0;i<sizeXs(head_xs);i++){
		fprintf(file_xs, "%.4f   %.4f   %.4f\n",current->xs.u_xs,current->xs.v_xs,current->xs.a_xs);
		current=current->next;
	}
	fclose(file_xs);
}

void writeVisionData(NodeVis *head_vis){
	FILE *file_vis=fopen("file_vision.txt","w");
	if(file_vis==NULL){
		printf("error writing data to file");
		exit(1);
	}
	NodeVis *current = head_vis;
	for(int i=0;i<sizeVis(head_vis);i++){
		fprintf(file_vis, "%.4f %.4f %.4f\n",current->vis.x_vis,current->vis.y_vis,current->vis.a_vis);
		current=current->next;
	}
	fclose(file_vis);
}


void EKF(){
  //initilize the head of the linkedlist of xsens and vision
  NodeXs *head_xs = (NodeXs*)malloc(sizeof(NodeXs*));
  NodeVis *head_vis = (NodeVis *)malloc(sizeof(NodeVis *));
  readVisionTxt(head_vis);
  readXsensTxt(head_xs);
	//printVis(head_vis);
  //printXs(head_xs);

  int f_xs=100,f_vis=50;
  float delay = 0.08;
  float delt_xs = (float)1/f_xs;
  int N_ahead = (int) (delay/delt_xs);

  float cn[2][2]={{var_z,0},{0,var_z}};
  float cu[3][3]={{var_a,0,0},{0,var_a,0},{0,0,var_theta}};
  float cw[4][4]={{var_xpos,0,0,0},{0,var_xpos,0,0},{0,0,var_xvel,0},{0,0,0,var_xvel}};
  float H[2][4]={{1,0,0,0},{0,1,0,0}};
  float x_pred[4][1]={0};
  float x_upd[4][1]={0};
  float cx_pred[4][4]={{1,0,0,0},{0,1,0,0},{0,0,0.1,0},{0,0,0,0.1}};
  float cx_upd[4][4]={0};
  float S[2][2]={0};
  float K[4][2]={0};
  float F[4][4]={{1,0,delt_xs,0},{0,1,0,delt_xs},{0,0,1,0},{0,0,0,1}};
  //delete the first N_ahead of vision
  for(int i=0;i<N_ahead;i++){
    popNodeVis(&head_vis);
  }

  //for(int k_index=0;k_index<sizeVis(head_vis)-N_ahead;k_index++){
    if(sizeVis(head_vis)<1){
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
      for(int i=0;i<2;i++){
        for(int j=0;j<4;j++){
          for(int k=0;k<4;k++){
            s1[i][j]+=H[i][k]*cx_pred[k][j];
          }
        }
      }

      //calculate (H*cx_pred)*H'
      for(int i=0;i<2;i++){
        for(int j=0;j<2;j++){
          S[i][j]=cn[i][j];
          for(int k=0;k<4;k++){
            S[i][j]+=s1[i][k]*H[j][k];
          }
        }
      }

      //*********************//
      // Kalman Gain Matrix  //
      //*********************//

      //calculate cx_pred*H'
      float k1[4][2]={0};
      for(int i=0;i<4;i++){
        for(int j=0;j<2;j++){
          for(int k=0;k<4;k++){
            k1[i][j]+=cx_pred[i][k]*H[j][k];
          }
        }
      }
      float index=1/(S[0][0]*S[1][1]-S[0][1]*S[1][0]);
      float s_inv[2][2]={{index*S[1][1],-index*S[0][1]},{-index*S[1][0],index*S[0][0]}};

      //calculate (cx_pred*H')*s^(-1)
      for(int i=0;i<4;i++){
        for(int j=0;j<2;j++){
          for(int k=0;k<2;k++){
            K[i][j]+=k1[i][k]*s_inv[k][j];
          }
        }
      }
      //end Kalman Gain Matrix

      //************************//
      // update state estimate  //
      //************************//
      //can be done for all the linked list data with for loop
      //need to be changed for all the nodes in the list
      float zk[2][1]={{head_vis->vis.x_vis},{head_vis->vis.y_vis}};
      float temp1[2][1]={0},temp2[2][1]={0};
      //calculate zk-H*x_pred
      for(int i=0;i<2;i++){
        for(int j=0;j<1;j++){
          for(int k=0;k<4;k++){
            temp1[i][j]+=H[i][k]*x_pred[k][j];
          }
          temp2[i][j]=zk[i][j]-temp1[i][j];
        }
      }

      float temp3[4][1]={0};
      //calculate x_pred+K*(zk-H*x_pred)
      for(int i=0;i<4;i++){
        for(int j=0;j<1;j++){
          for(int k=0;k<2;k++){
            temp3[i][j]+=K[i][k]*temp2[k][j];
          }
          x_upd[i][j]=x_pred[i][j]+temp3[i][j];
        }
      }
  //end state estimate

    //*****************************//
    // update covariance estimate  //
    //*****************************//
    //calculate K*S
    float temp4[4][2]={0};
    for(int i=0;i<4;i++){
      for(int j=0;j<2;j++){
        for(int k=0;k<2;k++){
          temp4[i][j]+=K[i][k]*S[k][j];
        }
      }
    }

    //calculate cx_pred-K*S*K'
    float temp5[4][4]={0};
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
        for(int k=0;k<2;k++){
          temp5[i][j]+=temp4[i][k]*K[j][k];
        }
      cx_upd[i][j]=cx_pred[i][j]-temp5[i][j];
      }
    }

    }//end if


    //************************//
    // prediction             //
    //************************//
    float u[3][1]={{head_xs->xs.u_xs},{head_xs->xs.v_xs},{head_xs->xs.a_xs}};
    //x_pred =F*x_upd+delt_xs*gfun(uk)//F(4,4),
    float c=cos(u[2][0]),s=sin(u[2][0]);
    float gfun[4][1]={{0},{0},{c*u[0][0]-s*u[1][0]},{c*u[1][0]+s*u[0][0]}};
    for(int i=0;i<4;i++){
      for(int j=0;j<1;j++){
        for(int k=0;k<4;k++){
          x_pred[i][j]+=F[i][k]*x_upd[k][j];
        }
        x_pred[i][j]+=delt_xs*gfun[i][j];
      }
    }

    float G[4][3]={{0,0,0},{0,0,0},{c,-s,-u[0][0]*s-u[1][0]*c},{s,c,u[0][0]*c-u[1][0]*s}};
    float temp_cx[4][4]={0};
    //calculate F*cx_upd
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
        for(int k=0;k<4;k++){
          temp_cx[i][j]+=F[i][k]*cx_upd[k][j];
        }
      }
    }

    //calculate G*cu
    float temp_G[4][3]={0};
    for(int i=0;i<4;i++){
      for(int j=0;j<3;j++){
        for(int k=0;k<3;k++){
          temp_G[i][j]+=G[i][k]*cu[k][j];
        }
      }
    }

    float cx_update_temp[4][4]={0};
    //calculate (F*cx_upd)*F'
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
        for(int k=0;k<4;k++){
          cx_update_temp[i][j]+=temp_cx[i][k]*F[j][k];
        }
      }
    }

    float G_cu_temp[4][4]={0};
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
        for(int k=0;k<3;k++){
          G_cu_temp[i][j]+=temp_G[i][k]*G[j][k];
        }
        cx_pred[i][j]=cx_update_temp[i][j]+G_cu_temp[i][j]+cw[i][j];
      }
    }
  //}//end for loop

}

int main(){
  EKF();
}
