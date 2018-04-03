#include <stdio.h>
#include <stdlib.h>
#include "xsens.h"

#define PI 3.141593

void writeXsensData(NodeXs *head_xs){
	FILE *file_xs=fopen("file_xsens.txt","w");
	if(file_xs==NULL){
		printf("error writing data to file");
		exit(1);
	}
	//struct xsens xs;
	NodeXs *current = head_xs;
	for(int i=0;i<sizeXs(head_xs);i++){
		// xs.u_xs=current->xs.u_xs;
		// xs.v_xs=current->xs.v_xs;
		// xs.a_xs=current->xs.a_xs;
		fprintf(file_xs, "%.4f %.4f %.4f\n",current->xs.u_xs,current->xs.v_xs,current->xs.a_xs);
		current=current->next;
	}
	fclose(file_xs);
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


int main() {
	NodeXs *head_xs = (NodeXs *)malloc(sizeof(NodeXs *));
	readXsensTxt(head_xs);
	writeXsensData(head_xs);
	// struct xsens xs;
	// NodeXs *current = head_xs;
	// for(int i=0;i<sizeXs(head_xs);i++){
	// 	xs.u_xs=current->xs.u_xs;
	// 	xs.v_xs=current->xs.v_xs;
	// 	xs.a_xs=current->xs.a_xs;
	// 	writeXsensData(xs);
	// 	current=current->next;
	// }
	return 0;
}
