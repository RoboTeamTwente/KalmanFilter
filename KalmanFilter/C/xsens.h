#include <stdio.h>
#include <stdlib.h>

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
