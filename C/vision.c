#include "vision.h"

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
    printf("%.6f %.6f\n",current->vis.x_vis,current->vis.y_vis);
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

void readVisionTxt(NodeVis *head_vis){
  FILE *myfile_vis;
	float x_vis,y_vis;
	struct vision vis,vis_init;
	myfile_vis = fopen("visionSyncedData50Hz.txt","r");
  fscanf(myfile_vis,"%f%f",&x_vis,&y_vis);
  vis_init.x_vis=x_vis;
  vis_init.y_vis=y_vis;
  // vis_init.a_vis=a_vis/180*PI;
	for(int i=0;i<1000;i++){
		fscanf(myfile_vis,"%f%f",&x_vis,&y_vis);
		vis.x_vis=x_vis-vis_init.x_vis;
		vis.y_vis=y_vis-vis_init.y_vis;
		// vis.a_vis=a_vis/180*PI-vis_init.a_vis;
		insertNodeVis(head_vis,vis);
	}
  fclose(myfile_vis);
}

void writeVisionData(NodeVis *head_vis){
	FILE *file_vis=fopen("file_vision.txt","w");
	if(file_vis==NULL){
		printf("error writing data to file");
		exit(1);
	}
	NodeVis *current = head_vis;
	for(int i=0;i<sizeVis(head_vis);i++){
		fprintf(file_vis, "%.4f %.4f \n",current->vis.x_vis,current->vis.y_vis);
		current=current->next;
	}
	fclose(file_vis);
}
