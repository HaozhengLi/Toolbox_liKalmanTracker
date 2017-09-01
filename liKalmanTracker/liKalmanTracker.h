/************************************************************/
/*  File Name: liKalmanTrack.h								*/
/*  Description: Class for multi-target Kalman tracking.	*/
/*  Author: Haozheng Li										*/
/*  EMail: 466739850@qq.com									*/
/************************************************************/

#include "liFunction.h"

class liKalmanFilter : public KalmanFilter
{
public:
	liKalmanFilter(int id_0 = 0, Point2f measurement_0 = Point2f(0, 0));
	~liKalmanFilter();

	int id;
	float confidence;
	vector<Point2f> trajectory;
	int inside_inc;
	int outside_inc;

	Point2f position();
	void confidenceIncrease();
	bool confidenceDecrease();

private:
	int confidence_inc;
	int confidence_dec;

};

class liKalmanTracker
{
public:
	liKalmanTracker(float targetSize_0 = 60, string targetName_0 = "target");
	~liKalmanTracker();

	vector<liKalmanFilter> target;
	float size;
	string name;
	int count;

	void track(vector<Point2f> measurement);
	vector<Point2f> trackment();
	void print(int frameCount = 0);
	Mat show(Mat src, int type = 0);

private:
	vector<bool> idTabel;

	void idTabelUpdate(int id);
	int idCreator();
	Scalar colorTabel(int id_0 = 0);

};
