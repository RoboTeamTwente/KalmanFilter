#include <stdio.h>

struct Array_2_1{
  float a[2][1];
};

struct Array_3_1{
  float a[3][1];
};

struct Array_2_2{
  float a[2][2];
};

struct Array_3_3{
  float a[3][3];
};

struct Array_4_4{
  float a[4][4];
};

struct Array_2_4{
  float a[2][4];
};

struct Array_4_2{
  float a[4][2];
};

struct Array_4_1{
  float a[4][1];
};

//S=H*cx_pred*H.transpose()+cn
struct Array_2_2 innovationMatrix(struct Array_2_4 H,struct Array_4_4 cx_pred,struct Array_2_2 cn){
  struct Array_2_4 temp1;
  for(int i=0;i<2;i++){
    for(int j=0;j<4;j++){
      temp1.a[i][j]=0;
      for(int k=0;k<4;k++){
        temp1.a[i][j]+=H.a[i][k]*cx_pred.a[k][j];
      }
    }
  }

  struct Array_2_2 temp2;
  for(int i=0;i<2;i++){
    for(int j=0;j<2;j++){
      temp2.a[i][j]=0;
      for(int k=0;k<4;k++){
        temp2.a[i][j]+=temp1.a[i][k]*H.a[j][k];
      }
    }
  }
  struct Array_2_2 S;
  for(int i=0;i<2;i++){
    for(int j=0;j<2;j++){
      S.a[i][j]=0;
      S.a[i][j]=temp2.a[i][j]+cn.a[i][j];
    }
  }
  return S;
}

//K=cx_pred*H.transpose()*inv(S)
struct Array_4_2 KalmanGainMatrix(struct Array_4_4 cx_pred,struct Array_2_4 H,struct Array_2_2 S){
  struct Array_4_2 temp1;//temp1=cx_pred*H.transpose()
  for(int i=0;i<4;i++){
    for(int j=0;j<2;j++){
      temp1.a[i][j]=0;
      for(int k=0;k<4;k++){
        temp1.a[i][j]+=cx_pred.a[i][k]*H.a[j][k];
      }
    }
  }

  struct Array_2_2 temp2;
  struct Array_4_2 K;
  float para=(S.a[0][0]*S.a[1][1]-S.a[0][1]*S.a[1][0]);
  if(para==0){}
  temp2.a[0][0]=1/para*S.a[1][1];
  temp2.a[0][1]=-1/para*S.a[0][1];
  temp2.a[1][0]=-1/para*S.a[1][0];
  temp2.a[1][1]=1/para*S.a[0][0];
  for(int i=0;i<4;i++){
    for(int j=0;j<2;j++){
      K.a[i][j]=0;
      for(int k=0;k<2;k++){
        K.a[i][j]+=temp1.a[i][k]*temp2.a[k][j];
      }
    }
  }
  return K;
}
//x_upd = x_pred+K*(zk-H*x_pred)
struct Array_4_1 stateExtimate(struct Array_4_1 x_pred,struct Array_4_2 K,struct Array_2_1 zk,struct Array_2_4 H){
  struct Array_2_1 temp1;
  for(int i=0;i<2;i++){
    for(int j=0;j<1;j++){
      temp1.a[i][j]=0;
      for(int k=0;k<4;k++){
        temp1.a[i][j]+=H.a[i][k]*x_pred.a[k][j];
      }
    }
  }
  struct Array_4_1 temp2;
  for(int i=0;i<4;i++){
    for(int j=0;j<1;j++){
      temp2.a[i][j]=0;
      for(int k=0;k<2;k++){
        temp2.a[i][j]+=x_pred.a[i][j]+K.a[i][k]*(zk.a[k][j]-temp1.a[k][j]);
      }
    }
  }
  return temp2;
}
//cx_upd=cx_pred-K*S*K.transpose()
struct Array_4_4 covarianceExtimate(struct Array_4_4 cx_pred,struct Array_4_2 K,struct Array_2_2 S){
  struct Array_4_2 temp1;
  for(int i=0;i<4;i++){
    for(int j=0;j<2;j++){
      temp1.a[i][j]=0;
      for(int k=0;k<2;k++){
        temp1.a[i][j]+=K.a[i][k]*S.a[k][j];
      }
    }
  }

  struct Array_2_1 temp2;
  for(int i=0;i<4;i++){
    for(int j=0;j<2;j++){
      temp2.a[i][j]=0;
      for(int k=0;k<4;k++){
        temp2.a[i][j]+=temp1.a[i][k]*K.a[j][k];
      }
    }
  }

  struct Array_4_4 cx_upd;
  for(int i=0;i<4;i++){
    for(int j=0;j<4;j++){
      cx_upd.a[i][j]=cx_pred.a[i][j]-temp2.a[i][j];
    }
  }
  return cx_upd;
}
