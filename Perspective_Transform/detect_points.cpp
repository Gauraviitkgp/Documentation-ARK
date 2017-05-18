#include <stdio.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

int i,j,t,p;

void displayimage(Mat image, String windowname){

    imshow(windowname, image);
}


Mat readimage(String filename)
{
    return imread(filename);
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << x << ","<< y<< endl;
     }
}




int main(int argc, char** argv){
  int i,j,k;
    Mat image=imread(argv[1],CV_LOAD_IMAGE_COLOR);

    namedWindow("ima",1);
    setMouseCallback("ima", CallBackFunc, NULL);
    imshow("ima",image);
    waitKey(0);
    return 0;
}
