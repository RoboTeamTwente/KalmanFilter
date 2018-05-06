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

void insertNodeXs(NodeXs *head,struct xsens xs);

void popNodeXs(NodeXs **head);

struct xsens get_kth_nodeXs(NodeXs *head_xs,int k);

void printXs(NodeXs *head);

int sizeXs(NodeXs *head);

void readXsensTxt(NodeXs *head_xs);

void writeXsensData(NodeXs *head_xs);
