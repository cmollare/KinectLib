#include "kinectInterface.h"
#include "viewerOpenCV.h"

void displayMenu();

int main(int argc, char* argv[])
{
	std::string oniFile = argv[1];
	std::string path = argv[1];
	path.erase(path.rfind('/')+1);

	std::cout << oniFile << std::endl;
	std::cout << path << std::endl;

	//kinect option : skeleton fitting + playback
	KinectInterface kinect("", true, oniFile);
	ViewerOpenCV viewer;

	KinectStruct data;
	KinectStruct* pData = &data;

	kinect.init();

	while(1)
	{
		kinect.updateAllMaps();
		kinect.getKinectData(pData);

		std::cout << "timestamp : " << pData->timestamp << std::endl;

		//record images in update
		viewer.update(data, path);

		if((cv::waitKey(25) != -1) || pData->isEOF) break;
	}

	return 0;
}

void displayMenu()
{
	using namespace std;

	cout << "Warning : no recording path specified" << endl;
	cout << "		   the stream is only played..." << endl;
	cout << "If you want to record type one of the following command" << endl;
	cout << "-ONI <record file path>" << endl;
	cout << "-AVI <record file path>" << endl;
}
