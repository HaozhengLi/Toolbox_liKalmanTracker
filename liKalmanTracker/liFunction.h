/************************************************************/
/*  File Name: liFunction.h									*/
/*  Description: Functions used in the program.				*/
/*  Author: Haozheng Li										*/
/*  EMail: 466739850@qq.com									*/
/************************************************************/

#include <iostream>
#include <sstream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;
using namespace cv;

//calculate distance p2p
float liDistance(Point2f pt1, Point2f pt2);

//calculate mean p2p
Point2f liMean(Point2f pt1, Point2f pt2);

//correct detected point, which should be in ROI and not too close
vector<Point2f> liCorrectDetection(vector<Point2f> pt, Rect roi, float dist_T);

//transfer Point2f to Mat
Mat liPointToMat(Point2f pt);

//transfet Mat to Point2f
Point2f liMatToPoint(Mat pt);

//get test target set
vector<vector<Point2f>> get_testTargetSet();

//get test detection set
vector<vector<Point2f>> get_testDetectionSet();
