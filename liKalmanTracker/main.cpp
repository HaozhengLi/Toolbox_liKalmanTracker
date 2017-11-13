/************************************************************/
/*  File Name: main.cpp										*/
/*  Description: A toolbox using OpenCV 2.4.9 to do point	*/
/*				 multi-target Kalman tracking.				*/
/*  Author: Haozheng Li										*/
/*  EMail: 466739850@qq.com									*/
/*  Created Time: 2017.5.25									*/
/*  Last Revised: 2017.11.13								*/
/*  Version: v1.0.4											*/
/************************************************************/

#include "liFunction.h"
#include "liKalmanTracker.h"

int main()
{	
	//Load the test data set
	vector<vector<Point2f>> testTargetSet = get_testTargetSet();
	vector<vector<Point2f>>::iterator p_testTargetSet = testTargetSet.begin();
	vector<vector<Point2f>> testDetectionSet = get_testDetectionSet();
	vector<vector<Point2f>>::iterator p_testDetectionSet = testDetectionSet.begin();

	//Set targets' size, name and ROI, load the Kalman tracker
	float targetSize = 40;
	string targetName = "person";
	Rect ROI = Rect(30, 5, 400, 250);
	liKalmanTracker tracker(targetSize, targetName);

	//Processing
	/************************************************************/
	/*    In practice, it's easy to change this program to deal	*/
	/*  with a video. Just change the while() loop and so on,	*/
	/*  then you got a multi-detect-tracking framework, COOL!!	*/
	/************************************************************/
	int nFrameCount = 1;
	Mat src, dst_detection, dst_measurement, dst_tracking;
	while (p_testTargetSet != testTargetSet.end() && p_testDetectionSet != testDetectionSet.end())
	{
		cout << nFrameCount << ":" << endl;
		src = Mat::zeros(300, 600, CV_8UC3);
		rectangle(src, ROI, Scalar(255, 255, 255), 1);

		//Target
		/************************************************************/
		/*    In practice, targets are usually heads, pedestrians,	*/
		/*  faces, etc. The source images are usually with three	*/
		/*  channels as RGB.										*/
		/************************************************************/
		vector<Point2f>::iterator target = p_testTargetSet->begin();
		while (target != p_testTargetSet->end())
		{
			circle(src, *target, 20, Scalar(255, 255, 255), CV_FILLED);
			target++;
		}
		namedWindow("Target", CV_WINDOW_AUTOSIZE);
		imshow("Target", src);

		//Detection:
		/************************************************************/
		/*    In practice, use classifiers like Haar + Adaboost or	*/
		/*  HOG + SVM to do detection. Then you can get the result	*/
		/*  of detection, which is type "vector<Point2f>" just like	*/
		/*	"p_testDetectionSet".									*/
		/************************************************************/
		dst_detection = src.clone();
		vector<Point2f>::iterator detection = p_testDetectionSet->begin();
		while (detection != p_testDetectionSet->end())
		{
			circle(dst_detection, *detection, 20, Scalar(0, 0, 255), 2);
			detection++;
		}
		namedWindow("Detection", CV_WINDOW_AUTOSIZE);
		imshow("Detection", dst_detection);

		//Measurement
		/************************************************************/
		/*    In practice, detection bounding boxes from one target	*/
		/*  may overlap to each other, or out of the ROI. Result of	*/
		/*  detection must be corrected as measurement, which is	*/
		/*  type "vector<Point2f>" just like "measurement".			*/
		/************************************************************/
		dst_measurement = src.clone();
		vector<Point2f> measurement = liCorrectDetection(*p_testDetectionSet, ROI, tracker.size);
		for (int i = 0; i < measurement.size(); i++)
		{
			circle(dst_measurement, measurement[i], 20, Scalar(0, 255, 0), 2);
		}
		namedWindow("Measurement", CV_WINDOW_AUTOSIZE);
		imshow("Measurement", dst_measurement);

		//Tracking
		/************************************************************/
		/*    In practice, just use the track(), print() and show()	*/
		/*  APIs to do multi-tracking. Or you can also use the		*/
		/*  trackment() API to directly get the result of tracking,	*/
		/*  whose type is "vector<Point2f>".						*/
		/************************************************************/
		dst_tracking = src.clone();
		tracker.track(measurement);
		tracker.print(nFrameCount);
		dst_tracking = tracker.show(dst_tracking, 0);
		namedWindow("Tracking", CV_WINDOW_AUTOSIZE);
		imshow("Tracking", dst_tracking);

		////Save
		//stringstream ssFrameCount;
		//string sFrameCount;
		//ssFrameCount << nFrameCount;
		//ssFrameCount >> sFrameCount;
		//string sFileName1 = "../result/Target/" + sFrameCount + ".jpg";
		//string sFileName2 = "../result/Detection/" + sFrameCount + ".jpg";
		//string sFileName3 = "../result/Measurement/" + sFrameCount + ".jpg";
		//string sFileName4 = "../result/Tracking/" + sFrameCount + ".jpg";
		//imwrite(sFileName1, src);
		//imwrite(sFileName2, dst_detection);
		//imwrite(sFileName3, dst_measurement);
		//imwrite(sFileName4, dst_tracking);

		//Next frame
		nFrameCount++;
		p_testTargetSet++;
		p_testDetectionSet++;

		//Video Control
		int nKey = waitKey(50);
		if (nKey == 27)	break;
		if (nKey == 32)
		{
			cout << "Pause." << endl;
			nKey = waitKey();
			if (nKey == 27)	break;
		}
	}
	
	return 1;
}
