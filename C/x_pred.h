#include<stdio.h>
#include<stdlib.h>

struct x_pred{
  float x[4][1];
};

typedef struct node_x_pred{
  struct x_pred data;
  struct node_x_pred *next;
}NodePred;

void insertNodePred(NodePred *head,float x[4][1]){
  NodePred *current = head;
  while(current->next!=NULL){
    current=current->next;
  }
  current->next = (NodePred*)malloc(sizeof(NodePred *));
  struct x_pred d;
  for(int i=0;i<4;i++){
    d.x[i][0]=x[i][0];
  }
  current->next->data=d;
  current->next->next=NULL;
}

void popNodePred(NodePred **head){
  NodePred *next_node = (*head)->next;
  free(*head);
  *head=next_node;
}

void writePredictedData(NodePred *head_x_pred){
	FILE *file_x_pred=fopen("x_pred.txt","w");
	if(file_x_pred==NULL){
		printf("error writing data to file");
		exit(1);
	}
	NodePred *current = head_x_pred;
	while(current->next!=NULL){
    fprintf(file_x_pred, "%f %f %f %f\n",
    head_x_pred->data.x[0][0],head_x_pred->data.x[1][0],
    head_x_pred->data.x[2][0],head_x_pred->data.x[3][0]);
    current=current->next;
  }
	fclose(file_x_pred);
}
