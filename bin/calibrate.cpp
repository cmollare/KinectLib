#include "kinectInterface.h"
#include "calibrateOpenCV.h"

void displayMenu();

int main(int argc, char* argv[])
{
	if (argc == 1)
		displayMenu();

	KinectInterface kinect("", false);
	CalibrateOpenCV viewer(9,6,25);

	KinectStruct data;
	KinectStruct* pData = &data;

	kinect.init();

	int state =0;

	while(1)
	{
		char key = cv::waitKey(25);

		switch (state)
		{
			case 0:
				kinect.updateAllMaps();
				kinect.getKinectData(pData);

				viewer.update(data);

				if (key == 'c')
					state=1;

				break;

			case 1:
				viewer.check(key);
				if (key == 'f')
					state=2;
				break;

			case 2:
				viewer.calibrate();
				return 0;
				break;

			default:
				break;
		}

		if(key == 'q') break;
	}

	return 0;
}

void displayMenu()
{
}
