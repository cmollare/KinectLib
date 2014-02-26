#include "calibrateOpenCV.h"

CalibrateOpenCV::CalibrateOpenCV(int nbRows, int nbCols, double squareSize)
{
	_data.nbRows = nbRows;
	_data.nbCols = nbCols;
	_data.squareSize = squareSize;

	_currentImage = 0;

	_update = true;

	cv::namedWindow("colorMap");
}

CalibrateOpenCV::~CalibrateOpenCV()
{
	cv::destroyAllWindows();
	cv::waitKey(30);
}

void CalibrateOpenCV::update(KinectStruct& kinectData)
{
	cv::Mat rgbMap;
	std::vector<cv::Point2f> detectedCorners;
	bool patternFound;

	rgbMap = cv::Mat(kinectData.RGBHeight, kinectData.RGBWidth, CV_8UC3, kinectData.imageMetaData.WritableData());
	cv::cvtColor(rgbMap, rgbMap, CV_BGR2RGB);

	patternFound = cv::findChessboardCorners(rgbMap, cv::Size(_data.nbRows, _data.nbCols), detectedCorners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

	if (patternFound)
	{
		cv::Mat gray;
		cv::cvtColor(rgbMap, gray, CV_BGR2GRAY);
		cv::cornerSubPix( gray, detectedCorners, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
		_images.push_back(rgbMap.clone());
		_detectedCorners.push_back(detectedCorners);
		//std::cout << detectedCorners << std::endl;
		cv::waitKey(500);

	}

	cv::drawChessboardCorners(rgbMap, cv::Size(_data.nbRows, _data.nbCols), detectedCorners, patternFound);
	cv::imshow("colorMap", rgbMap);
}

void CalibrateOpenCV::check(char key)
{
	if (_update)
	{
		cv::destroyWindow("colorMap");
		cv::namedWindow("colorMap");
		if (_currentImage == _images.size())
			_currentImage--;
		cv::createTrackbar("frame", "colorMap", &_currentImage, _images.size()-1);
		_update = false;
	}

	cv::imshow("colorMap", _images[_currentImage]);

	if (key == 'd' || key == 'r')
	{
		_images.erase(_images.begin()+_currentImage);
		_detectedCorners.erase(_detectedCorners.begin()+_currentImage);
		_update = true;
	}
}

void CalibrateOpenCV::calibrate()
{
	std::cout << "CAAAAAALIBRAAAATE" << std::endl;

	std::vector<cv::Point3f> realWorld;

	for (int j=0 ; j < _data.nbCols ; j++)
		for (int i=0 ; i < _data.nbRows ; i++)
		{

			int ind = i*_data.nbCols + j;
			realWorld.push_back(cv::Point3f(i*_data.squareSize, j*_data.squareSize, 0));
		}

	std::vector<std::vector<cv::Point3f> > objects(_detectedCorners.size(), realWorld);

	std::vector<cv::Mat> rVecs, tVecs;

	//std::cout << realWorld << std::endl;
	//std::cout << _detectedCorners[0] << std::endl;

	cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F);
	cv::Mat distCoeffs = cv::Mat::zeros(8,1,CV_64F);
	cv::calibrateCamera(objects, _detectedCorners, _images[0].size(), cameraMatrix, distCoeffs, rVecs, tVecs);

	std::cout << "matrice camera" << std::endl;
	std::cout << cameraMatrix << std::endl;
	std::cout << "distorsion" << std::endl;
	std::cout << distCoeffs << std::endl;

}


