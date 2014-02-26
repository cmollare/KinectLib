#ifndef CALIBRATEOPENCV_H
#define CALIBRATEOPENCV_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "kinectInterface.h"

typedef struct CalibData
{
	int nbRows;
	int nbCols;
	double squareSize;
}CalibData;


class CalibrateOpenCV
{
	public:
		CalibrateOpenCV(int nbRows, int nbCols, double squareSize);
		~CalibrateOpenCV();
		void update(KinectStruct& kinectData);
		void check(char key);
		void calibrate();


	protected:
		CalibData _data;
		std::vector<cv::Mat> _images;
		std::vector<std::vector<cv::Point2f> > _detectedCorners;
		int _currentImage;

		bool _update;
};

#endif
