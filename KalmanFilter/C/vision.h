#include <stdio.h>
#include <stdlib.h>

struct vision{
  float x_vis;
  float y_vis;
  float a_vis;
};

typedef struct node_vis{
  struct vision vis;
  struct node_vis *next;
}NodeVis;

void insertNodeVis(NodeVis *head,struct vision vis){
  NodeVis *current = head;
  while(current->next != NULL){
    current=current->next;
  }
  current->next = (NodeVis *)malloc(sizeof(NodeVis *));
  current->next->vis = vis;
  current->next->next=NULL;
}

void popNodeVis(NodeVis **head){
  NodeVis *next_node = (*head)->next;
  free(*head);
  *head = next_node;
}

struct vision get_kth_nodeVis(NodeVis *head_vis,int k){
  NodeVis *current = head_vis;
  for(int i=0;i<k;i++){
    current=current->next;
  }
  return current->vis;
}

void printVis(NodeVis *head){
  NodeVis *current = head;
  while(current->next != NULL){
    current=current->next;
    printf("%.6f %.6f %.6f\n",current->vis.x_vis,current->vis.y_vis,current->vis.a_vis);
  }
}

int sizeVis(NodeVis *head){
  NodeVis *current = head;
  int count=0;
  while(current->next != NULL){
    current=current->next;
    count++;
  }
  return count;
}
