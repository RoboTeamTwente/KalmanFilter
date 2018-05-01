#include <stdio.h>
#include <stdlib.h>

#define PI 3.141593

struct xsens{
  float u_xs;
  float v_xs;
  float a_xs;
};

typedef struct node_xs{
  struct xsens xs;
  struct node_xs *next;
}NodeXs;

void insertNodeXs(NodeXs *head,struct xsens xs){
  NodeXs *current = head;
  while(current->next != NULL){
    current = current->next;
  }
  current->next = (NodeXs *)malloc(sizeof(NodeXs *));
  current->next->xs = xs;
  current->next->next = NULL;
}

void popNodeXs(NodeXs **head){
  NodeXs *next_node = (*head)->next;
  free(*head);
  *head=next_node;
}

struct xsens get_kth_nodeXs(NodeXs *head_xs,int k){
  NodeXs *current=head_xs;
  for(int i=0;i<k;i++){
    current=current->next;
  }
  return current->xs;
}

void printXs(NodeXs *head){
  NodeXs *current = head;
  while(current->next != NULL){
    current=current->next;
    printf("%.6f %.6f %.6f\n",current->xs.u_xs,current->xs.v_xs,current->xs.a_xs);
  }
}

int sizeXs(NodeXs *head){
  NodeXs *current = head;
  int count=0;
  while(current->next != NULL){
    current=current->next;
    count++;
  }
  return count;
}

void readXsensTxt(NodeXs *head_xs){
  FILE *myfile_xs;
  float u_xs,v_xs,a_xs;
  struct xsens xs;
  myfile_xs = fopen("xsensSyncedData.txt","r");
  for(int i=0;i<2000;i++){
    fscanf(myfile_xs,"%f%f%f",&u_xs,&v_xs,&a_xs);
    xs.u_xs=u_xs;
    xs.v_xs=v_xs;
    xs.a_xs=a_xs;
    // xs.a_xs=a_xs/180*PI;
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
