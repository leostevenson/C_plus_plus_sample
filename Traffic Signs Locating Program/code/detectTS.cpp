/********************************************************************************
*
*
*  This program is demonstration for ellipse fitting. Program finds 
*  contours and approximate it by ellipses.
*
*  Trackbar specify threshold parametr.
*
*  White lines is contours. Red lines is fitting ellipses.
*
*
*  Autor:  Denis Burenkov.
*	Editor: Fan Zhang
*
*
********************************************************************************/

#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#endif

int slider_pos = 70;

//const CvSize img_size = cvSize(500, 500);
const int roiSize = 50;
const int scanInterval = 1;

boolean camera_mode = false;
boolean detect_corner = false;

#define MAX_TOL  150.00
#define ERROR_RATIO 0.07;
#define MIN_AREA 300
#define RATE_ERROR 0.5

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
//CvSize img_size;

using namespace std;

// Load the source image. HighGUI use.
IplImage *image = 0, *image00 = 0, *image01 = 0, *image02 = 0, *image03 = 0, *image04 = 0;

void handleImage();

void processImage(int h);

IplImage* GetThresholdedImage(IplImage* img);

boolean checkEllipseInContour(CvSeq* contour, CvPoint2D32f* PointArray2D32f, int count);

int main( int argc, char** argv )
{
	int i,j;
    const char* filename = argc == 2 ? argv[1] : (char*)"stuff.jpg";
    
    // Create windows.
    cvNamedWindow("Source", 1);
	cvNamedWindow("Threshed", 1);
	cvNamedWindow("Contour", 1);
	cvNamedWindow("Result", 1);

	if(argc==1){
		camera_mode = true;
		CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
		if ( !capture ) {
			fprintf( stderr, "ERROR: capture is NULL \n" );
			getchar();
			return -1;
		}	

		while ( 1 ) {
			// Get one frame
			image = cvQueryFrame( capture );
			if ( !image ) {
				fprintf( stderr, "ERROR: frame is null...\n" );
				getchar();
				break;
			}
			handleImage();
			cvShowImage( "camera", image );
			// Do not release the frame!
			//If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
			//remove higher bits using AND operator
			if ( (cvWaitKey(10) & 255) == 27 ) break;
		}
		cvDestroyWindow( "camera" );
	}
	else{
		// load image and force it to be grayscale
		if( (image = cvLoadImage(filename)) == 0 )
			return -1;

		handleImage();
	}
	
    cvDestroyWindow("Source");
    cvDestroyWindow("Threshed");
	cvDestroyWindow("Contour");
	cvDestroyWindow("Result");

    return 0;
}

void handleImage(){
	image00 = cvCreateImage(cvSize(500, 500), image->depth, image->nChannels);
	cvResize(image, image00);

	image01 = cvCloneImage(image00);
	image03 = cvCreateImage(cvGetSize(image01), image01->depth, 1);
	
	image03 = GetThresholdedImage(image01);
	//cvCvtColor(image01, image03, CV_RGB2GRAY);		

	cvShowImage("Threshed", image03);

	image02 = cvCloneImage( image03 );
	image04 = cvCloneImage( image03 );

	processImage(0);
			             
	cvResetImageROI(image03);

	cvReleaseImage(&image01);
	cvReleaseImage(&image02);
	cvReleaseImage(&image03);

	cvShowImage("Result", image);
}

// Define trackbar callback functon. This function find contours,
// draw it and approximate it by ellipses.
void processImage(int h)
{
    CvMemStorage* stor;
    CvSeq* cont;
    CvBox2D32f* box;
    CvPoint* PointArray;
    CvPoint2D32f* PointArray2D32f;
    
    // Create dynamic structure and sequence.
    stor = cvCreateMemStorage(0);
    cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
    
    // Threshold the source image. This needful for cvFindContours().
    //cvThreshold( image03, image02, slider_pos, 255, CV_THRESH_BINARY );
	cvThreshold( image03, image02, 1, 255, CV_THRESH_BINARY );
    
    // Find all contours.
    cvFindContours( image02, stor, &cont, sizeof(CvContour), 
                    CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
    

    // Clear images. IPL use.
    cvZero(image02);
    cvZero(image04);
    
	int j = 0;
	int nEllipse = 0;
    // This cycle draw all contours and approximate it by ellipses.
    for(;cont;cont = cont->h_next)
    {   
		j++;
        int i; // Indicator of cycle.
        int count = cont->total; // This is number point in contour
        
        // Number point must be more than or equal to 6 (for cvFitEllipse_32f).        
		if( count < 6 )
            continue;

        // Alloc memory for contour point set.    
        PointArray = (CvPoint*)malloc( count*sizeof(CvPoint) );
        PointArray2D32f= (CvPoint2D32f*)malloc( count*sizeof(CvPoint2D32f) );
                
        // Get contour point set.
        cvCvtSeqToArray(cont, PointArray, CV_WHOLE_SEQ);
        
        // Convert CvPoint set to CvBox2D32f set.
        for(i=0; i<count; i++)
        {
            PointArray2D32f[i].x = (float)PointArray[i].x;
            PointArray2D32f[i].y = (float)PointArray[i].y;
        }
		
        // Draw current contour.
        cvDrawContours(image04,cont,CV_RGB(255,255,255),CV_RGB(255,255,255),0,1, 8);

		cvShowImage( "Contour", image04 );
		
		if(!checkEllipseInContour(cont, PointArray2D32f, count)){
			//cvWaitKey(0);
			continue;
		}
		       
		nEllipse++;
        // Free memory.          
        free(PointArray);
        free(PointArray2D32f);
    }
    
    // Show image. HighGUI use.
    cvShowImage( "Result", image00 );
	
	cout << "Test: number of ellipses: " << nEllipse << endl;
	if(nEllipse>0){
		cvWaitKey(0);
	}
}


IplImage* GetThresholdedImage(IplImage* img)
{
	// Convert the image into an HSV image
	IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
	cvCvtColor(img, imgHSV, CV_BGR2HSV);

	IplImage* imgThreshed = cvCreateImage(cvGetSize(imgHSV), imgHSV->depth, 1);
	IplImage* imgThreshed1 = cvCreateImage(cvGetSize(imgHSV), imgHSV->depth, 1);
	IplImage* imgThreshed2 = cvCreateImage(cvGetSize(imgHSV), imgHSV->depth, 1);
	
	cvInRangeS(imgHSV, cvScalar(0, 50, 50), cvScalar(20, 255, 255), imgThreshed1);
	cvInRangeS(imgHSV, cvScalar(150, 50, 50), cvScalar(179, 255, 255), imgThreshed2);
	cvOr(imgThreshed1, imgThreshed2, imgThreshed);

	//cvInRangeS(imgHSV, CV_RGB(19, 13, 18), cvScalar(154, 256, 79), imgThreshed);
	//cvInRangeS(imgHSV, cvScalar(150, 50, 50), cvScalar(179, 255, 255), tmpImgThreshed2);

	//cvAdd(tmpImgThreshed1, tmpImgThreshed2, imgThreshed);

	//IplImage* imgNormal = cvCreateImage(cvGetSize(img), 8, 3);
	//cvCvtColor(imgThreshed, imgNormal, CV_GRAY2RGB);

	cvReleaseImage(&imgHSV);
	cvReleaseImage(&imgThreshed1);
	cvReleaseImage(&imgThreshed2);

	return imgThreshed;
}

boolean checkEllipseInContour(CvSeq* contour, CvPoint2D32f* PointArray2D32f, int count){

	int i;
	double actual_area = fabs(cvContourArea(contour, CV_WHOLE_SEQ, 0));
	if (actual_area < MIN_AREA){
		cout << "Test: not ellipse since acutal_area=" << actual_area << endl;
		return false;
	}

	CvRect rect = ((CvContour *)contour)->rect;
	int C = rect.width / 2; 
	int D = rect.height / 2;
	double estimated_area2 = M_PI * C * D;
	double error2 = fabs(actual_area - estimated_area2);  

	int th = (rect.width*rect.height)*ERROR_RATIO;
	if (error2 > th){
		cout << "Test: not ellipse since rectangle ellipse error=" << error2  << " [" << actual_area << "," << estimated_area2 << "," << th << "]" << endl;
		return false;
	}

	if(!(C>D*RATE_ERROR & C<D)){
		cout << "Test: not ellipse since ratio_error_rate: " << "[C,D] = [" << C << "," << D << "]" << endl;
		return false;
	}
	
	//Corner detection
	if(detect_corner){
		IplImage *tmpImage0 = cvCreateImage(cvGetSize(image03), image03->depth, image03->nChannels);
		cvDrawContours(tmpImage0,contour,CV_RGB(255,255,255),CV_RGB(255,255,255),0,1, 8);
		cvSetImageROI(tmpImage0, rect);
		IplImage *tmpImage1 = cvCreateImage(cvGetSize(tmpImage0), tmpImage0->depth, tmpImage0->nChannels);
		cvCopy(tmpImage0, tmpImage1, NULL);
		cvResetImageROI(tmpImage0);


		IplImage *tmpImage2 = cvCreateImage(cvGetSize(tmpImage1), tmpImage1->depth, tmpImage1->nChannels);
		IplImage *tmpImage3 = cvCreateImage(cvGetSize(tmpImage1), tmpImage1->depth, tmpImage1->nChannels);
		const int MAX_CORNERS = 900;
		CvPoint2D32f* corners;
		corners = (CvPoint2D32f*)malloc( MAX_CORNERS*sizeof(CvPoint2D32f) );
		int corner_count = MAX_CORNERS;
		double quality_level = 10;
		double min_distance = rect.width;
		int eig_block_size = rect.width;
		int use_harris = true;
		//cvResize(image, tmpImage1);
		//cvCvtColor(tmpImage, tmpImage2, CV_RGB2GRAY);
		//IplImage *tmpImage3 = cvCloneImage(tmpImage2);
		cvGoodFeaturesToTrack(image03, tmpImage2, tmpImage3, corners, &corner_count, quality_level, min_distance, NULL, eig_block_size, use_harris);

		if(corner_count > 0){
			cout << "Test: not ellipse since corner # = : "  << corner_count << endl;
			return false;
		}
	}

	cvRectangle(image00, cvPoint(rect.x, rect.y), cvPoint(rect.x+rect.width, rect.y+rect.height), cvScalar(0xff,0x00,0x00), 3);
	cvShowImage( "Source", image00 );
	cout << "Test: found ellipse in [" << PointArray2D32f[0].x << "," << PointArray2D32f[0].y << "]" << endl;
	return true;
	
}

#ifdef _EiC
main(1,"fitellipse.c");
#endif