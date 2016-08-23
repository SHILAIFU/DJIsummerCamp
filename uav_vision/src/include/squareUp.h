/*
 * SquareUp.h
 *
 *  Created on: 2016-8-2
 *      Author: ros-alg-2
 */

#ifndef SUQARE_UP__H_
#define SUQARE_UP__H_

#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;
using namespace std;

cv::Scalar GridColor[3] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 255), cv::Scalar(0, 0, 255) };

class SquareUp
{
public:
    void init(double myBlueThres = 170, double myRedThres = 170, double myYellowThres = 200, int myProdThres = 3, string squareParameter = "/home/dreamer/VisionPackage-master/markerDetect/data/squareParameter.yml")	//200
	{
		blueThres = myBlueThres;
		redThres = myRedThres;
		yellowThres = myYellowThres;
		prodThres = myProdThres;

		blueHsvThres = new int(6);
		redHsvThres = new int(6);
		yellowHsvThres = new int(6);

        cv::FileStorage fs;
        fs.open(squareParameter, cv::FileStorage::READ);

		blueHsvThres[0] = fs["blueHThresMin"];
		blueHsvThres[1] = fs["blueHThresMax"];
		blueHsvThres[2] = fs["blueSThresMin"];
		blueHsvThres[3] = fs["blueSThresMax"];
		blueHsvThres[4] = fs["blueVThresMin"];
		blueHsvThres[5] = fs["blueVThresMax"];

		redHsvThres[0] = fs["redHThresMin"];
		redHsvThres[1] = fs["redHThresMax"];
		redHsvThres[2] = fs["redSThresMin"];
		redHsvThres[3] = fs["redSThresMax"];
		redHsvThres[4] = fs["redVThresMin"];
		redHsvThres[5] = fs["redVThresMax"];

		yellowHsvThres[0] = fs["yellowHThresMin"];
		yellowHsvThres[1] = fs["yellowHThresMax"];
		yellowHsvThres[2] = fs["yellowSThresMin"];
		yellowHsvThres[3] = fs["yellowSThresMax"];
		yellowHsvThres[4] = fs["yellowVThresMin"];
		yellowHsvThres[5] = fs["yellowVThresMax"];
	}

	/*
	 * function: ���Ź��񣬷���ÿ����ɫ�ķ����λ��
	 * @src input rgbͼ������
	 * @return 3����ɫ�ķ����λ�ã���һά����ɫ���ֱ�Ϊ�����ơ��죬�ڶ�ά�Ƕ�Ӧ��ɫ�ķ����λ������
	 */
    vector< vector<cv::Point> > detect(cv::Mat src)
	{
        cv::Mat srcCopy;
		src.copyTo(srcCopy);

		//��ɫ�ռ�ת��
		Mat hsv, blue, red1, red2, red, yellow, white;
		cvtColor(srcCopy, hsv, CV_BGR2HSV);
		inRange(hsv, Scalar(0, 0, 221), Scalar(180, 30, 255), white);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(white, white, cv::MORPH_CLOSE, element);

		imshow("white", white);

        vector<cv::Mat> rgb;
		split(srcCopy, rgb);

        cv::Mat g, r;
        cv::threshold(rgb[1], g, yellowThres, 255, cv::THRESH_BINARY);
        cv::threshold(rgb[0], r, 150, 255, cv::THRESH_BINARY);
		g = g&r;

        cv::Mat edge;
        cv::Canny(white, edge, 50, 200, 3);
        cv::imshow("edge", edge);

        vector<cv::Vec4i> linesP;
        cv::HoughLinesP(edge, linesP, 1, CV_PI / 180, 80, srcCopy.cols / 8, srcCopy.cols / 2);

        for( size_t i = 0; i < linesP.size(); i++ )
        {
        	line( srcCopy, Point(linesP[i][0], linesP[i][1]), Point(linesP[i][2], linesP[i][3]), Scalar(255, 255, 0), 5, 8 );
        	//angle constrait
        	float ang = atan2(linesP[i][3] - linesP[i][1], linesP[i][2] - linesP[i][0]) * 180.f/3.14159;
        	if(abs(ang) < 20 || abs(ang - 90) < 20 || abs(ang - 180) < 20 || abs(ang - 270) < 20)
        	{
				if((linesP[i][0] - linesP[i][2])*(linesP[i][1] - linesP[i][3]) < 0)
				{
					int yCoe, xCoe;
					if(linesP[i][1] > 10 && linesP[i][3] > 10)
						yCoe = 10;
					else
						yCoe = 0;
					if(linesP[i][0] > 10 && linesP[i][2] > 10)
						xCoe = 10;
					else
						xCoe = 0;
					float x = (linesP[i][1]*linesP[i][2] - linesP[i][0]*linesP[i][3] + yCoe*(linesP[i][0] - linesP[i][2]))/(float)(linesP[i][1] - linesP[i][3]);
					float y = (linesP[i][3]*linesP[i][0] - linesP[i][2]*linesP[i][1] + xCoe*(linesP[i][1] - linesP[i][3]))/(float)(linesP[i][0] - linesP[i][2]);
					cv::line(g, cv::Point2f(x, 0), cv::Point2f(0, y), cv::Scalar(255), 5, 8);
				}
				else if((linesP[i][0] - linesP[i][2])*(linesP[i][1] - linesP[i][3]) > 0)
				{
					int yCoe, xCoe;
					if(linesP[i][1] > 10 && linesP[i][3] > 10)
						yCoe = 10;
					else
						yCoe = 0;
					if(linesP[i][1] < 470 && linesP[i][3] < 470)
						xCoe = 470;
					else
						xCoe = 480;
					float m = (linesP[i][1]*linesP[i][2] - linesP[i][0]*linesP[i][3] + yCoe*(linesP[i][0] - linesP[i][2]))/(float)(linesP[i][1] - linesP[i][3]);
					float n = (linesP[i][1]*linesP[i][2] - linesP[i][0]*linesP[i][3] + xCoe*(linesP[i][0] - linesP[i][2]))/(float)(linesP[i][1] - linesP[i][3]);
					cv::line(g, cv::Point2f(m, 0), cv::Point2f(n, 480), cv::Scalar(255), 5, 8);
				}
				else if(linesP[i][0] == linesP[i][2])
				{
					cv::line(g, cv::Point(linesP[i][0], 0), cv::Point(linesP[i][0], src.rows - 1), cv::Scalar(255), 5, 8);
				}
				else if(linesP[i][1] == linesP[i][3])
				{
					cv::line(g, cv::Point(0, linesP[i][1]), cv::Point(src.cols - 1, linesP[i][1]), cv::Scalar(255), 5, 8);
				}
        	}
        }
        imshow("line", white);

        vector< vector<cv::Point> > contours;
        vector< vector<cv::Point> > squares;
		findContours(g, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        vector<cv::Point> approx;
		for (size_t i = 0; i < contours.size(); i++)
		{
			//��������
            cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);

			//����
//			drawContours(src, contours, i, Scalar(0, 255, 0), 3, 8, vector<Vec4i>(), 0, Point());
//			stringstream ss;
//			ss << approx.size();
//			putText(src, ss.str(), approx[0], 1, 3, Scalar(255, 0, 0));
//			imshow("src", src);

            if (approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > 500 && cv::isContourConvex(cv::Mat(approx)))	//&& fabs(contourArea(Mat(approx))) > 1000
			{
				double maxCosine = 0;

				for (int j = 2; j < 5; j++)
				{
					double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
					maxCosine = MAX(maxCosine, cosine);
				}

				if (maxCosine < 0.5)
				{
					Rect r = boundingRect(approx);
					squares.push_back(approx);
				}
			}
		}
		cout << squares.size() << endl;

//		vector<Point3i> grid = ColorDetect(squares, rgb);
        vector< vector<cv::Point> > grid = ColorDetect(squares, srcCopy);
		draw(src, srcCopy, squares);
		DrawColor(srcCopy, grid);

        cv::imshow("g", g);
        cv::imshow("srcCopy", srcCopy);
		return grid;
	}

	/**********���Ƽ����**************/
    void draw(cv::Mat src, cv::Mat& dst, vector< vector<cv::Point> > squares)
	{
		src.copyTo(dst);

		for (size_t i = 0; i < squares.size(); i++)
		{
//			drawContours(dst, squares, i, Scalar(0, 255, 0), 3, 8, vector<Vec4i>(), 0, Point());
            cv::RotatedRect minRect = cv::minAreaRect(cv::Mat(squares[i]));
			double h2w = minRect.size.height>minRect.size.width?minRect.size.height/minRect.size.width:minRect.size.width/minRect.size.height;
			if(h2w < 3)
			{
                cv::Point2f rect_points[4];
				minRect.points(rect_points);
				for (int j = 0; j < 4; j++)
				{
                    cv::line(dst, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(0, 0, 255), 3, 8); //
				}
			}
		}
	}

    void DrawColor(cv::Mat& src, vector<cv::Point3i> grid)
	{
		for(size_t i=0;i<grid.size();i++)
		{
            cv::circle(src, cv::Point(grid[i].x, grid[i].y), 10, GridColor[grid[i].z], 3);
		}
	}

    void DrawColor(cv::Mat& src, vector< vector<cv::Point> > grid)
	{
		for(size_t i=0;i<3;i++)
		{
			for(size_t j=0;j<grid[i].size();j++)
			{
                circle(src, cv::Point(grid[i][j].x, grid[i][j].y), 10, GridColor[i], 3);
			}
		}
	}
private:
    bool isSquareUp(vector<cv::Vec4i> linesP)
	{
        vector<cv::Point> linesVec;
		for(size_t i=0;i<linesP.size();i++)
		{
            cv::Point linesTemp = cv::Point(linesP[i][0], linesP[i][1]) - cv::Point(linesP[i][2], linesP[i][3]);
			linesVec.push_back(linesTemp);
		}
		int paraNum = 0, vertNum = 0;
		for(size_t i=0;i<linesVec.size();i++)
		{
			for(size_t j=0;j<linesVec.size();j++)
			{
				if(i!=j)
				{
					int innerProd = linesVec[i].dot(linesVec[j]);
					int crossProd = linesVec[i].cross(linesVec[j]);
					if(innerProd < prodThres)
					{
						vertNum++;
					}
					if(crossProd < prodThres)
					{
						paraNum++;
					}
					if(paraNum > 2 && vertNum > 2 && paraNum < 8 && vertNum < 8)
						return true;
				}
			}
			paraNum = 0;
			vertNum = 0;
		}
		return false;
	}

	/****************�ж�ÿ���������ɫRGB****************/
    vector< vector<cv::Point> > ColorDetect(vector< vector<cv::Point> > squares, vector<cv::Mat> rgb)
	{
        cv::Mat green;
		rgb[1].copyTo(green);
        cv::threshold(rgb[2], rgb[2], redThres, 255, cv::THRESH_BINARY);
        cv::threshold(rgb[0], rgb[0], blueThres, 255, cv::THRESH_BINARY);
        cv::threshold(rgb[1], rgb[1], 150, 255, cv::THRESH_BINARY);
		rgb[1] = rgb[1] & rgb[2];
        cv::threshold(green, green, 100, 255, cv::THRESH_BINARY_INV);
		rgb[2] = rgb[2] & green;

//		imshow("rgb[0]", rgb[0]);
//		imshow("rgb[1]", rgb[1]);
//		imshow("rgb[2]", green);

        vector< vector<cv::Point> > colorGrid(3, vector<cv::Point>());
		for(size_t i = 0; i < squares.size(); i++)
		{
            cv::Rect r = cv::boundingRect(squares[i]);
            float h2w = r.height>r.width?r.height/(float)r.width:r.width/(float)r.height;
			if(h2w < 3)
			{
				for(size_t j=0;j<3;j++)
				{
                    cv::Mat roi = rgb[j](r);
					int area = 0;
					for(int m=0;m<roi.rows;m++)
					{
						for(int n=0;n<roi.cols;n++)
						{
							if(roi.at<uchar>(m, n) == 255)
								area++;
						}
					}

					if((float)area/r.area() > 0.5)
					{
                        colorGrid[j].push_back(cv::Point(r.x + r.width / 2, r.y + r.height / 2));
						continue;
					}
				}
			}
		}
		return colorGrid;
	}

	/****************�ж�ÿ���������ɫHSV****************/
    vector< vector<cv::Point> > ColorDetect(vector< vector<cv::Point> > squares, cv::Mat src)
	{
		//��ɫ�ռ�ת��
        cv::Mat hsv, blue, red1, red2, yellow;
		cvtColor(src, hsv, CV_BGR2HSV);
        vector<cv::Mat> h_s_v1, h_s_v2, h_s_v3, h_s_v4;
        cv::Mat h_s_v[3] = { cv::Mat(), cv::Mat(), cv::Mat() };

        cv::inRange(hsv, cv::Scalar(blueHsvThres[0], blueHsvThres[2], blueHsvThres[4]), cv::Scalar(blueHsvThres[1], blueHsvThres[3], blueHsvThres[5]), blue);
        cv::split(blue, h_s_v1);
		h_s_v[0] = h_s_v1[0];
		imshow("blue", h_s_v1[0]);

        cv::inRange(hsv, cv::Scalar(yellowHsvThres[0], yellowHsvThres[2], yellowHsvThres[4]), cv::Scalar(yellowHsvThres[1], yellowHsvThres[3], yellowHsvThres[5]), yellow);
        cv::split(yellow, h_s_v2);
		h_s_v[1] = h_s_v2[0];
		imshow("yellow", h_s_v2[0]);

        cv::inRange(hsv, cv::Scalar(redHsvThres[0], redHsvThres[2], redHsvThres[4]), cv::Scalar(redHsvThres[1], redHsvThres[3], redHsvThres[5]), red1);
        cv::inRange(hsv, cv::Scalar(156, redHsvThres[2], redHsvThres[4]), cv::Scalar(180, redHsvThres[3], redHsvThres[5]), red2);
        cv::split(red1, h_s_v3);
        cv::split(red2, h_s_v4);
		h_s_v3[0] = h_s_v3[0] | h_s_v4[0];
		h_s_v[2] = h_s_v3[0];
		imshow("red", h_s_v3[0]);

		imshow("blue", h_s_v[0]);
		imshow("yellow", h_s_v[1]);
		imshow("red", h_s_v[2]);

        vector< vector<cv::Point> > colorGrid(3, vector<cv::Point>());
		for(size_t i = 0; i < squares.size(); i++)
		{
            cv::Rect r = cv::boundingRect(squares[i]);
            float h2w = r.height>r.width?r.height/(float)r.width:r.width/(float)r.height;
			if(h2w < 3)
			{
				for(size_t j=0;j<3;j++)
				{
                    cv::Mat roi = h_s_v[j](r).clone();
					int area = 0;
					for(int m=0;m<roi.rows;m++)
					{
						for(int n=0;n<roi.cols;n++)
						{
							if(roi.at<uchar>(m, n) == 255)
								area++;
						}
					}

					if((float)area/r.area() > 0.6)
					{
                        colorGrid[j].push_back(cv::Point(r.x + r.width / 2, r.y + r.height / 2));
						continue;
					}
				}
			}
		}
		return colorGrid;
	}

	/***********��������ֱ�ߵļн�**************/
    double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
	{
		double dx1 = pt1.x - pt0.x;
		double dy1 = pt1.y - pt0.y;
		double dx2 = pt2.x - pt0.x;
		double dy2 = pt2.y - pt0.y;
		return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
	}

private:
	double blueThres;
	double redThres;
	double yellowThres;

	int prodThres;

	int* blueHsvThres;
	int* redHsvThres;
	int* yellowHsvThres;
};

#endif /* SUQARE_UP__H_ */
