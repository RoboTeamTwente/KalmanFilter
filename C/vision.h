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

void insertNodeVis(NodeVis *head,struct vision vis);
void popNodeVis(NodeVis **head);
struct vision get_kth_nodeVis(NodeVis *head_vis,int k);
void printVis(NodeVis *head);
int sizeVis(NodeVis *head);
void readVisionTxt(NodeVis *head_vis);
void writeVisionData(NodeVis *head_vis);
