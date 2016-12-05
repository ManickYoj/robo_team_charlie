//! [includes]
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace std;

string photo ="./src/RoboTeamCharlie/Square.jpg";
int identifySquares(){
	//adapted from http://opencv-srf.blogspot.com/2011/09/object-detection-tracking-using-contours.html
	IplImage* img =  cvLoadImage(photo); //loading the image
	//showing the original image
 	cvNamedWindow("Raw");
 	cvShowImage("Raw",img);
 	//namedWindow( "Raw");// Create a window for display.
    //imshow( "Raw", img );                   // Show our image inside it
 	//convert to greyscale to see contours
 	IplImage* imgGrayScale = cvCreateImage(cvGetSize(img), 8, 1); 
 	cvCvtColor(img,imgGrayScale,CV_BGR2GRAY);

 	//thresholding the grayscale image to get better results
 	cvThreshold(imgGrayScale,imgGrayScale,128,255,CV_THRESH_BINARY);  
 	CvSeq* contours;  //hold the pointer to a contour in the memory block
 	CvSeq* result;   //hold sequence of points of a contour
 	CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours

 	//finding all contours in the image
 	 cvFindContours(imgGrayScale, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
 	 //looping through all contours
 	 int count=0;
 	 while(contours){
 	 	cout <<contours << std::endl;

 	 //obtain a sequence of points of contour, pointed by the variable 'contour'
     result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0);
 	  {if(result->total==4 )
     	{
         //iterating through each point
         CvPoint *pt[4];
         for(int i=0;i<4;i++){
             pt[i] = (CvPoint*)cvGetSeqElem(result, i);
         }
         //drawing lines around the quadrilateral
         cvLine(img, *pt[0], *pt[1], cvScalar(255,0,0),4);
         cvLine(img, *pt[1], *pt[2], cvScalar(255,0,0),4);
         cvLine(img, *pt[2], *pt[3], cvScalar(255,0,0),4);
         cvLine(img, *pt[3], *pt[0], cvScalar(255,0,0),4);  
     	}
 	  }
 	   {if(result->total==3 )
     	{
         //iterating through each point
         CvPoint *pt[3];
         for(int i=0;i<3;i++){
             pt[i] = (CvPoint*)cvGetSeqElem(result, i);
         }
         //drawing lines around the quadrilateral
         cvLine(img, *pt[0], *pt[1], cvScalar(0,255,0),3);
         cvLine(img, *pt[1], *pt[2], cvScalar(0,255,0),3);
         cvLine(img, *pt[2], *pt[3], cvScalar(0,255,0),3);   
     	}
 	  }
 	  count=count+1;
 	  //obtain the next contour
     contours = contours->h_next;}
	//show the image in which identified shapes are marked   
 	cvNamedWindow("Tracked");
 	cvShowImage("Tracked",img);
   
 	cvWaitKey(0); //wait for a key press

 	//cleaning up
 	cvDestroyAllWindows(); 
 	cvReleaseMemStorage(&storage);
 	cvReleaseImage(&img);
 	cvReleaseImage(&imgGrayScale);

 	return 0;
}


int main( int argc, char** argv )
{
    if( argc != 2)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << std::endl;
     return -1;
    }

    Mat image;
     image = imread(photo, CV_LOAD_IMAGE_COLOR);   //s Read the filec
    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

//    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
//    imshow( "Display window", image );                   // Show our image inside it.

    identifySquares();
    waitKey(0);                                         // Wait for a keystroke in the window
    return 0;

}