/************************************************************/
/*  File Name: liFunction.cpp								*/
/*  Description: Functions used in the program.				*/
/*  Author: Haozheng Li										*/
/*  EMail: 466739850@qq.com									*/
/************************************************************/

#include "liFunction.h"

float liDistance(Point2f pt1, Point2f pt2)
{
	float s1 = pow(pt1.x - pt2.x, 2);
	float s2 = pow(pt1.y - pt2.y, 2);
	float dist = sqrt(s1 + s2);

	return dist;
}

Point2f liMean(Point2f pt1, Point2f pt2)
{
	Point2f pt(pt1.x / 2 + pt2.x / 2, pt1.y / 2 + pt2.y / 2);

	return pt;
}

vector<Point2f> liCorrectDetection(vector<Point2f> pt, Rect roi, float dist_T)
{
	vector<Point2f> pt_c;
	for (vector<Point2f>::iterator p = pt.begin(); p != pt.end(); p++)
	{
		if (p->x > roi.x && p->x < roi.x + roi.width && p->y > roi.y && p->y < roi.y + roi.height)
		{
			vector<Point2f>::iterator q = p + 1;
			for (; q != pt.end(); q++)
			{
				float dist = liDistance(*p, *q);
				if (dist < dist_T)
				{
					*q = liMean(*p, *q);
					break;
				}
			}

			if (q == pt.end())
			{
				pt_c.push_back(*p);
			}
		}
	}

	return pt_c;
}

Mat liPointToMat(Point2f pt)
{
	Mat pt_m = *(Mat_<float>(2, 1) << pt.x, pt.y);

	return pt_m;
}

Point2f liMatToPoint(Mat pt)
{
	Point2f pt_p(0, 0);
	
	if (pt.rows == 2 && pt.cols == 1)
	{
		pt_p.x = pt.at<float>(0);
		pt_p.y = pt.at<float>(1);
	}

	return pt_p;
}

vector<vector<Point2f>> get_testTargetSet()
{
	vector<vector<Point2f>> list;
	vector<Point2f> target;
	
	//1
	target.clear();
	target.push_back(Point2f(60, 50));

	target.push_back(Point2f(280, 270));
	list.push_back(target);

	//2
	target.clear();
	target.push_back(Point2f(80, 54));

	target.push_back(Point2f(275, 271));
	list.push_back(target);

	//3
	target.clear();
	target.push_back(Point2f(112, 52));

	target.push_back(Point2f(59, 96));

	target.push_back(Point2f(58, 140));

	target.push_back(Point2f(276, 260));
	list.push_back(target);

	//4
	target.clear();
	target.push_back(Point2f(122, 57));

	target.push_back(Point2f(79, 88));

	target.push_back(Point2f(88, 142));

	target.push_back(Point2f(277, 251));
	list.push_back(target);

	//5
	target.clear();
	target.push_back(Point2f(142, 52));

	target.push_back(Point2f(99, 78));

	target.push_back(Point2f(128, 140));

	target.push_back(Point2f(270, 249));
	list.push_back(target);

	//6
	target.clear();
	target.push_back(Point2f(162, 50));

	target.push_back(Point2f(120, 61));

	target.push_back(Point2f(156, 144));

	target.push_back(Point2f(272, 240));
	list.push_back(target);

	//7
	target.clear();
	target.push_back(Point2f(190, 55));

	target.push_back(Point2f(142, 53));

	target.push_back(Point2f(189, 142));

	target.push_back(Point2f(274, 242));
	list.push_back(target);

	//8
	target.clear();
	target.push_back(Point2f(220, 60));

	target.push_back(Point2f(172, 43));

	target.push_back(Point2f(219, 140));

	target.push_back(Point2f(275, 230));
	list.push_back(target);

	//9
	target.clear();
	target.push_back(Point2f(250, 67));

	target.push_back(Point2f(202, 23));

	target.push_back(Point2f(247, 145));

	target.push_back(Point2f(265, 233));
	list.push_back(target);

	//10
	target.clear();
	target.push_back(Point2f(272, 68));

	target.push_back(Point2f(229, 25));

	target.push_back(Point2f(269, 146));

	target.push_back(Point2f(245, 210));
	list.push_back(target);

	//11
	target.clear();
	target.push_back(Point2f(295, 78));

	target.push_back(Point2f(259, 30));

	target.push_back(Point2f(259, 142));

	target.push_back(Point2f(235, 190));
	list.push_back(target);

	//12
	target.clear();
	target.push_back(Point2f(320, 82));

	target.push_back(Point2f(279, 50));

	target.push_back(Point2f(257, 143));

	target.push_back(Point2f(215, 178));
	list.push_back(target);

	//13
	target.clear();
	target.push_back(Point2f(344, 89));

	target.push_back(Point2f(299, 68));

	target.push_back(Point2f(250, 142));

	target.push_back(Point2f(225, 153));
	list.push_back(target);

	//14
	target.clear();
	target.push_back(Point2f(374, 96));

	target.push_back(Point2f(320, 88));

	target.push_back(Point2f(252, 144));

	target.push_back(Point2f(229, 130));
	list.push_back(target);

	//15
	target.clear();
	target.push_back(Point2f(384, 104));

	target.push_back(Point2f(320, 119));

	target.push_back(Point2f(242, 142));

	target.push_back(Point2f(209, 110));
	list.push_back(target);

	//16
	target.clear();
	target.push_back(Point2f(404, 114));

	target.push_back(Point2f(350, 139));

	target.push_back(Point2f(222, 140));

	target.push_back(Point2f(189, 80));
	list.push_back(target);

	//17
	target.clear();
	target.push_back(Point2f(424, 134));

	target.push_back(Point2f(370, 159));

	target.push_back(Point2f(202, 142));

	target.push_back(Point2f(169, 62));
	list.push_back(target);

	//18
	target.clear();
	target.push_back(Point2f(446, 137));

	target.push_back(Point2f(390, 179));

	target.push_back(Point2f(182, 139));

	target.push_back(Point2f(159, 35));
	list.push_back(target);

	//19
	target.clear();
	target.push_back(Point2f(476, 146));

	target.push_back(Point2f(426, 199));

	target.push_back(Point2f(152, 143));

	target.push_back(Point2f(129, 25));
	list.push_back(target);

	//20
	target.clear();
	target.push_back(Point2f(489, 152));

	target.push_back(Point2f(456, 239));

	target.push_back(Point2f(115, 142));

	target.push_back(Point2f(99, 30));
	list.push_back(target);

	//21
	target.clear();
	target.push_back(Point2f(489, 152));

	target.push_back(Point2f(476, 259));

	target.push_back(Point2f(95, 140));

	target.push_back(Point2f(74, 39));
	list.push_back(target);
	
	//22
	target.clear();
	target.push_back(Point2f(496, 153));

	target.push_back(Point2f(499, 279));

	target.push_back(Point2f(75, 141));

	target.push_back(Point2f(54, 59));
	list.push_back(target);

	//23
	target.clear();
	target.push_back(Point2f(509, 155));

	target.push_back(Point2f(529, 289));

	target.push_back(Point2f(61, 141));

	target.push_back(Point2f(34, 79));
	list.push_back(target);

	//24
	target.clear();
	target.push_back(Point2f(529, 155));

	target.push_back(Point2f(559, 299));

	target.push_back(Point2f(41, 144));

	target.push_back(Point2f(14, 78));
	list.push_back(target);

	//25
	target.clear();
	target.push_back(Point2f(549, 157));

	target.push_back(Point2f(589, 309));

	target.push_back(Point2f(31, 144));

	target.push_back(Point2f(4, 72));
	list.push_back(target);

	//26
	target.clear();
	target.push_back(Point2f(579, 158));

	target.push_back(Point2f(600, 319));

	target.push_back(Point2f(11, 142));

	target.push_back(Point2f(-11, 73));
	list.push_back(target);

	//27
	target.clear();
	target.push_back(Point2f(599, 162));

	target.push_back(Point2f(600, 319));

	target.push_back(Point2f(-10, 141));

	target.push_back(Point2f(-33, 75));
	list.push_back(target);

	//28
	target.clear();
	target.push_back(Point2f(610, 165));

	target.push_back(Point2f(600, 319));

	target.push_back(Point2f(-30, 141));

	target.push_back(Point2f(-33, 75));
	list.push_back(target);

	//29
	target.clear();
	target.push_back(Point2f(625, 168));

	target.push_back(Point2f(600, 319));

	target.push_back(Point2f(-55, 142));

	target.push_back(Point2f(-33, 75));
	list.push_back(target);

	//30
	target.clear();
	target.push_back(Point2f(642, 168));

	target.push_back(Point2f(600, 319));

	target.push_back(Point2f(-55, 142));

	target.push_back(Point2f(-33, 75));
	list.push_back(target);

	return list;
}

vector<vector<Point2f>> get_testDetectionSet()
{
	vector<vector<Point2f>> list;
	vector<Point2f> target;

	//1
	target.clear();
	target.push_back(Point2f(64, 52));
	target.push_back(Point2f(66, 53));

	target.push_back(Point2f(283, 272));
	list.push_back(target);

	//2
	target.clear();
	target.push_back(Point2f(84, 52));

	target.push_back(Point2f(274, 272));
	target.push_back(Point2f(277, 273));
	list.push_back(target);

	//3
	target.clear();
	target.push_back(Point2f(110, 51));

	target.push_back(Point2f(58, 98));
	target.push_back(Point2f(59, 96));

	target.push_back(Point2f(52, 142));

	target.push_back(Point2f(275, 261));
	target.push_back(Point2f(274, 264));
	target.push_back(Point2f(273, 264));
	list.push_back(target);

	//4
	target.clear();
	target.push_back(Point2f(124, 56));
	target.push_back(Point2f(126, 57));

	target.push_back(Point2f(78, 86));

	target.push_back(Point2f(89, 140));

	target.push_back(Point2f(279, 250));
	target.push_back(Point2f(277, 252));
	list.push_back(target);

	//5
	target.clear();
	target.push_back(Point2f(144, 54));

	target.push_back(Point2f(102, 74));
	target.push_back(Point2f(103, 75));
	target.push_back(Point2f(104, 72));

	target.push_back(Point2f(126, 144));

	target.push_back(Point2f(272, 249));
	target.push_back(Point2f(274, 248));
	list.push_back(target);

	//6
	target.clear();
	target.push_back(Point2f(163, 51));

	target.push_back(Point2f(123, 63));

	target.push_back(Point2f(156, 142));
	target.push_back(Point2f(157, 143));

	target.push_back(Point2f(271, 242));
	list.push_back(target);

	//7
	target.clear();
	target.push_back(Point2f(194, 55));

	target.push_back(Point2f(141, 56));
	target.push_back(Point2f(143, 58));

	target.push_back(Point2f(186, 142));

	target.push_back(Point2f(271, 243));
	target.push_back(Point2f(273, 242));
	list.push_back(target);

	//8
	target.clear();
	target.push_back(Point2f(220, 61));

	target.push_back(Point2f(174, 43));
	target.push_back(Point2f(175, 42));
	target.push_back(Point2f(176, 41));
	target.push_back(Point2f(176, 42));

	target.push_back(Point2f(213, 142));
	target.push_back(Point2f(212, 141));

	target.push_back(Point2f(270, 231));
	list.push_back(target);

	//9
	target.clear();
	target.push_back(Point2f(252, 69));

	target.push_back(Point2f(203, 25));
	target.push_back(Point2f(204, 26));

	target.push_back(Point2f(245, 143));

	target.push_back(Point2f(264, 232));
	list.push_back(target);

	//10
	target.clear();
	target.push_back(Point2f(274, 69));

	target.push_back(Point2f(231, 27));

	target.push_back(Point2f(268, 143));

	target.push_back(Point2f(244, 212));
	target.push_back(Point2f(245, 213));
	list.push_back(target);

	//11
	target.clear();
	target.push_back(Point2f(296, 79));

	target.push_back(Point2f(258, 32));

	target.push_back(Point2f(258, 143));
	target.push_back(Point2f(257, 142));
	target.push_back(Point2f(256, 141));

	target.push_back(Point2f(233, 192));
	list.push_back(target);

	//12
	target.clear();
	target.push_back(Point2f(321, 83));
	target.push_back(Point2f(322, 84));

	target.push_back(Point2f(278, 52));

	target.push_back(Point2f(256, 141));
	target.push_back(Point2f(259, 145));

	target.push_back(Point2f(214, 176));
	list.push_back(target);

	//13
	target.clear();
	target.push_back(Point2f(343, 88));
	target.push_back(Point2f(348, 78));

	target.push_back(Point2f(297, 67));

	target.push_back(Point2f(251, 140));
	target.push_back(Point2f(258, 145));

	target.push_back(Point2f(224, 155));
	list.push_back(target);

	//14
	target.clear();
	target.push_back(Point2f(375, 98));

	target.push_back(Point2f(322, 89));
	target.push_back(Point2f(328, 79));

	target.push_back(Point2f(254, 142));

	target.push_back(Point2f(228, 131));
	list.push_back(target);

	//15
	target.clear();
	target.push_back(Point2f(386, 105));

	target.push_back(Point2f(322, 117));
	target.push_back(Point2f(332, 127));

	target.push_back(Point2f(241, 143));

	target.push_back(Point2f(208, 111));
	list.push_back(target);

	//16
	target.clear();
	target.push_back(Point2f(406, 115));

	target.push_back(Point2f(352, 137));
	target.push_back(Point2f(362, 136));

	target.push_back(Point2f(223, 142));

	target.push_back(Point2f(187, 82));
	list.push_back(target);

	//17
	target.clear();
	target.push_back(Point2f(422, 133));

	target.push_back(Point2f(168, 63));
	list.push_back(target);

	//18
	target.clear();
	target.push_back(Point2f(447, 135));

	target.push_back(Point2f(156, 32));
	list.push_back(target);

	//19
	target.clear();
	target.push_back(Point2f(477, 145));

	target.push_back(Point2f(127, 26));
	list.push_back(target);

	//20
	target.clear();
	target.push_back(Point2f(487, 153));

	target.push_back(Point2f(457, 238));
	target.push_back(Point2f(458, 232));

	target.push_back(Point2f(114, 143));

	target.push_back(Point2f(97, 32));
	list.push_back(target);

	//21
	target.clear();
	target.push_back(Point2f(487, 154));

	target.push_back(Point2f(478, 257));
	target.push_back(Point2f(472, 254));
	target.push_back(Point2f(471, 267));

	target.push_back(Point2f(96, 142));

	target.push_back(Point2f(73, 37));
	list.push_back(target);

	//22
	target.clear();
	target.push_back(Point2f(497, 154));

	target.push_back(Point2f(498, 278));

	target.push_back(Point2f(74, 142));
	target.push_back(Point2f(64, 132));

	target.push_back(Point2f(51, 58));
	list.push_back(target);

	//23
	target.clear();
	target.push_back(Point2f(507, 156));

	target.push_back(Point2f(524, 284));

	target.push_back(Point2f(66, 143));
	target.push_back(Point2f(65, 149));

	target.push_back(Point2f(35, 78));
	target.push_back(Point2f(45, 70));
	list.push_back(target);

	//24
	target.clear();
	target.push_back(Point2f(527, 153));
	target.push_back(Point2f(517, 152));
	target.push_back(Point2f(517, 150));

	target.push_back(Point2f(555, 296));

	target.push_back(Point2f(45, 142));

	target.push_back(Point2f(15, 79));
	list.push_back(target);

	//25
	target.clear();
	target.push_back(Point2f(548, 155));

	target.push_back(Point2f(587, 307));
	target.push_back(Point2f(577, 307));

	target.push_back(Point2f(33, 145));

	target.push_back(Point2f(2, 76));
	list.push_back(target);

	//26
	target.clear();
	target.push_back(Point2f(578, 159));

	target.push_back(Point2f(12, 144));
	list.push_back(target);

	//27
	target.clear();
	target.push_back(Point2f(599, 162));

	target.push_back(Point2f(600, 319));

	list.push_back(target);

	//28
	target.clear();

	target.push_back(Point2f(600, 319));

	list.push_back(target);

	//29
	target.clear();

	target.push_back(Point2f(600, 319));

	list.push_back(target);

	//30
	target.clear();

	target.push_back(Point2f(600, 319));

	list.push_back(target);

	return list;
}
