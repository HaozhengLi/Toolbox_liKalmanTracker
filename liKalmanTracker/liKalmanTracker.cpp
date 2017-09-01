/************************************************************/
/*  File Name: liKalmanTrack.h								*/
/*  Description: Class for multi-target Kalman tracking.	*/
/*  Author: Haozheng Li										*/
/*  EMail: 466739850@qq.com									*/
/************************************************************/

#include "liKalmanTracker.h"

liKalmanFilter::liKalmanFilter(int id_0, Point2f measurement_0)
{
	id = id_0;
	confidence = 4;
	trajectory.clear();
	inside_inc = 0;
	outside_inc = 0;
	confidence_inc = 0;
	confidence_dec = 0;

	statePre = Mat::zeros(4, 1, CV_32F);
	statePost = Mat::zeros(4, 1, CV_32F);	//x
	statePost.at<float>(0) = measurement_0.x;
	statePost.at<float>(1) = measurement_0.y;

	transitionMatrix = Mat::eye(4, 2, CV_32F);	//A
	transitionMatrix = *(Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
	controlMatrix.release();	//B
	measurementMatrix = Mat::zeros(2, 4, CV_32F);	//H
	setIdentity(measurementMatrix);
	gain = Mat::zeros(4, 2, CV_32F);	//K

	errorCovPre = Mat::zeros(4, 4, CV_32F);
	errorCovPost = Mat::zeros(4, 4, CV_32F);	//P
	setIdentity(errorCovPost, Scalar::all(1));
	processNoiseCov = Mat::eye(4, 4, CV_32F);	//Q
	setIdentity(processNoiseCov, Scalar::all(1e-2));
	measurementNoiseCov = Mat::eye(2, 2, CV_32F);	//R
	setIdentity(measurementNoiseCov, Scalar::all(1e-1));

	temp1.create(4, 4, CV_32F);
	temp2.create(2, 4, CV_32F);
	temp3.create(2, 2, CV_32F);
	temp4.create(2, 4, CV_32F);
	temp5.create(2, 1, CV_32F);
}

liKalmanFilter::~liKalmanFilter() {}

Point2f liKalmanFilter::position()
{
	Point2f pt(statePost.at<float>(0), statePost.at<float>(1));

	return pt;
}

void liKalmanFilter::confidenceIncrease()
{
	confidence_inc++;
	confidence_dec = 0;
	confidence += log(confidence_inc + 1) / log(1.5f);
}

bool liKalmanFilter::confidenceDecrease()
{
	confidence_dec++;
	confidence_inc = 0;
	confidence -= pow(1.5f, confidence_dec);
	if (confidence < 0)
	{
		confidence = 0;
		return false;
	}

	return true;
}

liKalmanTracker::liKalmanTracker(float targetSize_0, string targetName_0)
{
	target.clear();
	size = targetSize_0;
	name = targetName_0;
	count = target.size();
	idTabel.push_back(true);	//id 0 is always used
}

liKalmanTracker::~liKalmanTracker() {}

void liKalmanTracker::track(vector<Point2f> measurement)
{
	int measureCount = measurement.size();
	count = target.size();
	if (!measureCount && !count)
		return;

	if (count)
	{
		//too far m2k
		for (int i = 0; i < measureCount; i++)
		{
			float dist_min = liDistance(measurement[i], target[0].position());
			for (int j = 1; j < count; j++)
			{
				float dist = liDistance(measurement[i], target[j].position());
				if (dist < dist_min)
					dist_min = dist;
			}

			if (dist_min > 3 * size)
				target.push_back(liKalmanFilter(idCreator(), measurement[i]));
		}
	}
	count = target.size();

	if (count < measureCount)
	{
		//match
		for (int i = 0; i < count; i++)
		{
			int label = i;
			float dist_min = liDistance(target[i].position(), measurement[i]);
			for (int j = i + 1; j < measureCount; j++)
			{
				float dist = liDistance(target[i].position(), measurement[j]);
				if (dist < dist_min)
				{
					dist_min = dist;
					label = j;
				}
			}
			target[i].confidenceIncrease();
			swap(measurement[label], measurement[i]);

			target[i].predict();
			target[i].correct(liPointToMat(measurement[i]));
		}

		//none match
		for (int i = count; i < measureCount; i++)
		{
			target.push_back(liKalmanFilter(idCreator(), measurement[i]));

			target[i].predict();
			target[i].correct(liPointToMat(measurement[i]));
		}

		count = target.size();
	}
	else
	{
		//match
		for (int i = 0; i < measureCount; i++)
		{
			int label = i;
			float dist_min = liDistance(measurement[i], target[i].position());
			for (int j = i + 1; j < count; j++)
			{
				float dist = liDistance(measurement[i], target[j].position());
				if (dist < dist_min)
				{
					dist_min = dist;
					label = j;
				}
			}
			target[label].confidenceIncrease();
			swap(target[label], target[i]);

			target[i].predict();
			target[i].correct(liPointToMat(measurement[i]));
		}		

		//none match
		bool deleteTarget0 = false;
		for (vector<liKalmanFilter>::iterator k = target.begin() + measureCount; k != target.end(); k++)
		{
			if (!(*k).confidenceDecrease())
			{
				if (k != target.begin())
				{
					vector<liKalmanFilter>::iterator kt = k;
					k--;
					idTabelUpdate((*kt).id);
					target.erase(kt);
				}
				else
					deleteTarget0 = true;

				continue;
			}
			(*k).predict();
			measurement.push_back((*k).position());

			(*k).correct(liPointToMat(measurement[measurement.size() - 1]));
		}
		if (deleteTarget0)
		{
			idTabelUpdate(target[0].id);
			target.erase(target.begin());
		}

		count = target.size();
		//measureCount = measurement.size();
	}

}

vector<Point2f> liKalmanTracker::trackment()
{
	vector<Point2f> trackment;
	for (int i = 0; i < target.size(); i++)
		trackment.push_back(target[i].position());

	return trackment;
}

void liKalmanTracker::print(int frameCount)
{
	cout << endl;
	cout << "©°©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©´" << endl;
	cout << "©¦   " << left << setw(6) << frameCount << " ©¦ count ";
	cout << left << setw(6) << count;
	cout << "                               ©¦" << endl;
	if (idTabel.size() == 1)
	{
		cout << "©¸©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼" << endl;
		return;
	}
	cout << "©À©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È" << endl;
	cout << "©¦    id    ©¦      position      ©¦      confidence      ©¦" << endl;
	cout << "©À©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È" << endl;

	for (int i = 1; i < idTabel.size(); i++)
	{
		cout << "©¦    " << left << setw(6) << i << "©¦";
		if (idTabel[i])
		{
			int label = 0;
			for (int j = 0; j < count; j++)
			{
				if (target[j].id == i)
					label = j;
			}
			cout << "[" << right << fixed << setw(8) << setprecision(2) << target[label].position().x << ",";
			cout << right << fixed << setw(8) << setprecision(2) << target[label].position().y << " ]";
			cout << "©¦" << right << fixed << setw(16) << target[label].confidence << "      ©¦" << endl;
		}
		else
			cout << "        ----        ©¦         ----         ©¦" << endl;

		if (i != idTabel.size() - 1)
			cout << "©À©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È" << endl;
		else
			cout << "©¸©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼" << endl;
	}

}

Mat liKalmanTracker::show(Mat src, int type_0)
{
	Mat dst = src.clone();
	for (int i = 0; i < target.size(); i++)
	{
		target[i].trajectory.push_back(target[i].position());
		if (target[i].trajectory.size() > 500)
			target[i].trajectory.pop_back();

		vector<Point2f>::const_iterator pt = target[i].trajectory.end() - 1;
		Scalar color = colorTabel(target[i].id);
		circle(dst, *pt, 5, color, 2);
		stringstream ssid;
		string sid;
		ssid << target[i].id;
		ssid >> sid;
		stringstream ssconf;
		string sconf;
		ssconf << target[i].confidence;
		ssconf >> sconf;
		string st = name + "[" + sid + "]" + sconf;
		if (!type_0)
			circle(dst, *pt, size / 2, color, 2);
		else
			rectangle(dst, Point((*pt).x - size / 2, (*pt).y - size / 2), Point((*pt).x + size / 2, (*pt).y + size / 2), color, 2);
		putText(dst, st, Point((*pt).x - size / 2, (*pt).y + size * 3 / 4), FONT_HERSHEY_SIMPLEX, 0.5, color, 1);

		while (pt != target[i].trajectory.begin())
		{
			circle(dst, *(pt - 1), 3, color, 1);
			line(dst, *(pt - 1), *pt, color, 1);
			pt--;
		}
	}

	return dst;
}

void liKalmanTracker::idTabelUpdate(int id)
{
	idTabel[id] = false;

	while (!idTabel[idTabel.size() - 1])
		idTabel.pop_back();
}

int liKalmanTracker::idCreator()
{
	int id = 0;
	while (idTabel[id])
	{
		id++;
		if (id == idTabel.size())
		{
			idTabel.push_back(true);
			break;
		}
	}
	idTabel[id] = true;

	return id;
}

Scalar liKalmanTracker::colorTabel(int id_0)
{
	Scalar color(0, 255, 0);
	switch (id_0)
	{
	case 0:
	case 1: break;
	case 2: color = Scalar(0, 255, 255); break;
	case 3: color = Scalar(255, 255, 0); break;
	case 4: color = Scalar(255, 0, 255); break;

	case 5: color = Scalar(18, 153, 255); break;
	case 6: color = Scalar(230, 224, 176); break;
	case 7: color = Scalar(132, 227, 255); break;
	case 8: color = Scalar(240, 32, 160); break;
	case 9: color = Scalar(203, 192, 255); break;

	case 10: color = Scalar(3, 97, 255); break;
	case 11: color = Scalar(42, 42, 128); break;
	case 12: color = Scalar(34, 139, 34); break;
	case 13: color = Scalar(255, 144, 30); break;
	case 14: color = Scalar(80, 127, 255); break;
	case 15: color = Scalar(201, 252, 189); break;

	case 16: color = Scalar(221, 160, 221); break;
	case 17: color = Scalar(0, 215, 255); break;
	case 18: color = Scalar(85, 142, 235); break;
	case 19: color = Scalar(87, 38, 135); break;
	case 20: color = Scalar(140, 180, 210); break;
	default: break;
	}

	return color;
}
