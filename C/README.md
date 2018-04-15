compile the code with the following command:
gcc -o EKF EKF.c vision.h xsens.h Array.h x_pred.h -lm

The Array.h contains Array multiplication,summation and  substraction

The xsens.h contains xsens data reading, writing and insert and pop node in linkedlist

The vision.h contains camera data reading, writing and insert and pop node in linkedlist

The x_pred.h contains prediction data
