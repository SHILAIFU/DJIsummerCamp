/*
 * SquareUp.h
 *
 *  Created on: 2016-8-2
 *      Author: ros-alg-2
 */

#ifndef SquareUp_H_
#define SquareUp_H_

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Scalar GridColor[3] = {Scalar(255, 0, 0), Scalar(0, 255, 255), Scalar(0, 0, 255)};

class SquareUp
{
public:
    void init(double myBlueThres = 170, double myRedThres = 170, double myYellowThres = 200, int myProdThres = 3)	//200
    {
        blueThres = myBlueThres;
        redThres = myRedThres;
        yellowThres = myYellowThres;
        prodThres = myProdThres;
    }

    /*
     * function: 检测九宫格，返回每种颜色的方格的位置
     * @src input rgb图像输入
     * @return 3种颜色的方格的位置，第一维是颜色，分别为蓝、黄、红，第二维是对应颜色的方格的位置坐标
     */
    vector< vector<Point> > detect(Mat src, Mat &dst)

    {
        Mat srcCopy;
        src.copyTo(srcCopy);

        vector<Mat> rgb;
        split(srcCopy, rgb);

        //阈值分割
        Mat matG, matR;
        threshold(rgb[1], matG, yellowThres, 255, THRESH_BINARY);
        threshold(rgb[0], matR, 150, 255, THRESH_BINARY);
        matG = matG&matR;

        //边缘提取
        Mat edge;
        Canny(matG, edge, 50, 200, 3);
        // imshow("edge", edge);

        //形态学运算

        //直线拟合
        vector<Vec4i> linesP;
        HoughLinesP(edge, linesP, 1, CV_PI/180, 80, srcCopy.cols/8, srcCopy.cols/2);

        for( size_t i = 0; i < linesP.size(); i++ )
        {
            if((linesP[i][0] - linesP[i][2])*(linesP[i][1] - linesP[i][3]) < 0)
            {
                float x = (linesP[i][1]*linesP[i][2] - linesP[i][0]*linesP[i][3] + 2*(linesP[i][0] - linesP[i][2]))/(float)(linesP[i][1] - linesP[i][3]);
                float y = (linesP[i][3]*linesP[i][0] - linesP[i][2]*linesP[i][1] + 2*(linesP[i][1] - linesP[i][3]))/(float)(linesP[i][0] - linesP[i][2]);
                line(matG, Point2f(x, 0), Point2f(0, y), Scalar(255), 5, 8);
            }
            else if((linesP[i][0] - linesP[i][2])*(linesP[i][1] - linesP[i][3]) > 0)
            {
                float m = (linesP[i][1]*linesP[i][2] - linesP[i][0]*linesP[i][3] + 2*(linesP[i][0] - linesP[i][2]))/(float)(linesP[i][1] - linesP[i][3]);
                float n = (linesP[i][1]*linesP[i][2] - linesP[i][0]*linesP[i][3] + 480*(linesP[i][0] - linesP[i][2]))/(float)(linesP[i][1] - linesP[i][3]);
                line(matG, Point2f(m, 0), Point2f(n, 480), Scalar(255), 5, 8);
            }
            else if(linesP[i][0] == linesP[i][2])
            {
                line(matG, Point(linesP[i][0], 0), Point(linesP[i][0], src.rows - 1), Scalar(255), 5, 8);
            }
            else if(linesP[i][1] == linesP[i][3])
            {
                line(matG, Point(0, linesP[i][1]), Point(src.cols - 1, linesP[i][1]), Scalar(255), 5, 8);
            }
        }

        //轮廓提取
        vector< vector<Point> > contours;
        vector< vector<Point> > squares;
        findContours(matG, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        vector<Point> approx;
        for (size_t i = 0; i < contours.size(); i++)
        {
            //多边形拟合
            approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
            if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 500 && isContourConvex(Mat(approx)))	//&& fabs(contourArea(Mat(approx))) > 1000
            {
                double maxCosine = 0;

                for (int j = 2; j < 5; j++)
                {
                    double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                    maxCosine = MAX(maxCosine, cosine);
                }

                if (maxCosine < 0.5)
                {
                    squares.push_back(approx);
                }
            }
        }
        // cout << squares.size() << endl;
        draw(src, srcCopy, squares);

        vector< vector<Point> > grid = colorDetect(squares, rgb);
        drawColor(srcCopy, grid);

        // imshow("g", matG);
        // imshow("srcCopy", srcCopy);
        srcCopy.copyTo(dst);

        return grid;
    }

    /**********绘制检测结果**************/
    void draw(Mat src, Mat& dst, vector< vector<Point> > squares)
    {
        src.copyTo(dst);

        for (size_t i = 0; i < squares.size(); i++)
        {
            //			drawContours(dst, squares, i, Scalar(0, 255, 0), 3, 8, vector<Vec4i>(), 0, Point());
            RotatedRect minRect = minAreaRect(Mat(squares[i]));
            double h2w = minRect.size.height>minRect.size.width?minRect.size.height/minRect.size.width:minRect.size.width/minRect.size.height;
            if(h2w < 3)
            {
                Point2f rect_points[4];
                minRect.points(rect_points);
                for (int j = 0; j < 4; j++)
                {
                    line(dst, rect_points[j], rect_points[(j + 1) % 4], Scalar(0, 0, 255), 3, 8); //
                }
            }
        }
    }

    /************绘制九宫格的颜色块*************/
    void drawColor(Mat& src, vector<Point3i> grid)
    {
        for(size_t i=0;i<grid.size();i++)
        {
            circle(src, Point(grid[i].x, grid[i].y), 10, GridColor[grid[i].z], 3);
        }
    }

    void drawColor(Mat& src, vector< vector<Point> > grid)
    {
        for(size_t i=0;i<3;i++)
        {
            for(size_t j=0;j<grid[i].size();j++)
            {
                circle(src, Point(grid[i][j].x, grid[i][j].y), 10, GridColor[i], 3);
            }
        }
    }

private:
    /*********根据颜色块的个数和分布判断是否为九宫格***********/
    /*
         * 特征： 蓝：红：黄 = 1:1:1 and number（蓝，红，黄）∈[2, 3]
         * 红：（0， 0， 255）， 蓝：（255， 0， 0）， 黄：（0， 255， 255）
         * 问题： 1.投入的公仔挡住了部分区域
         * 		2.摄像头看不到一个完整的九宫格
         */
    bool isSquareUp(vector<Vec4i> linesP)
    {
        vector<Point> linesVec;
        for(size_t i=0;i<linesP.size();i++)
        {
            Point linesTemp = Point(linesP[i][0], linesP[i][1]) - Point(linesP[i][2], linesP[i][3]);
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

    /****************判断每个方格的颜色****************/
    vector< vector<Point> > colorDetect(vector< vector<Point> > squares, vector<Mat> rgb)
    {
        Mat green;
        rgb[1].copyTo(green);
        threshold(rgb[2], rgb[2], redThres, 255, THRESH_BINARY);
        threshold(rgb[0], rgb[0], blueThres, 255, THRESH_BINARY);
        threshold(rgb[1], rgb[1], 150, 255, THRESH_BINARY);
        rgb[1] = rgb[1] & rgb[2];
        threshold(green, green, 100, 255, THRESH_BINARY_INV);
        rgb[2] = rgb[2] & green;

        vector<Point3i> gridColor;
        vector< vector<Point> > colorGrid(3, vector<Point>());
        for(size_t i = 0; i < squares.size(); i++)
        {
            Rect r = boundingRect(squares[i]);
            float h2w = r.height>r.width ? (float)r.height/r.width : (float) r.width/r.height;
            if(h2w < 3)
            {
                for(size_t j=0;j<3;j++)
                {
                    Mat roi = rgb[j](r);
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
                        Point3i gridTemp;
                        gridTemp.x = r.x + r.width/2;
                        gridTemp.y = r.y + r.height/2;
                        gridTemp.z = j;
                        gridColor.push_back(gridTemp);

                        colorGrid[j].push_back(Point(r.x + r.width/2, r.y + r.height/2));
                        continue;
                    }
                }
            }
        }
        return colorGrid;
    }

    /***********计算两条直线的夹角**************/
    double angle(Point pt1, Point pt2, Point pt0)
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
};

#endif /* SquareUp_H_ */
