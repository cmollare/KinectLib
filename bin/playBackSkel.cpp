#include "kinectInterface.h"
#include "viewerOpenCV.h"

#include <fstream>

void displaySkel(KinectStruct *data, std::string path);

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

		displaySkel(pData, path);

		std::cout << "timestamp : " << pData->timestamp << std::endl;

		//record images in update
		viewer.update(data);

		if((cv::waitKey(25) != -1) || pData->isEOF) break;
	}

	return 0;

}

void displaySkel(KinectStruct* data, std::string path)
{
	std::string fileName = "/joints.txt";
	fileName = path+fileName;
	std::ofstream file(fileName.c_str(), std::ofstream::out | std::ofstream::app);

/*	std::vector<std::string> nameList, XYZ;
	nameList.push_back("Head");
	nameList.push_back("Neck");
	nameList.push_back("Torso");
	nameList.push_back("Waist");
	nameList.push_back("LCollar");
	nameList.push_back("LShoulder");
	nameList.push_back("LElbow");
	nameList.push_back("LWrist");
	nameList.push_back("LHand");
	nameList.push_back("LFingertip");
	nameList.push_back("RCollar");
	nameList.push_back("RShoulder");
	nameList.push_back("RElbow");
	nameList.push_back("RWrist");
	nameList.push_back("RHand");
	nameList.push_back("RFingertip");
	nameList.push_back("LHip");
	nameList.push_back("LKnee");
	nameList.push_back("LAnkle");
	nameList.push_back("LFoot");
	nameList.push_back("RHip");
	nameList.push_back("RKnee");
	nameList.push_back("RAnkle");
	nameList.push_back("RFoot");

	XYZ.push_back("X");
	XYZ.push_back("Y");
	XYZ.push_back("Z");*/

	file << data->timestamp;

	for (int i=0 ; i<data->jointPositions[0].size() ; i++)
	{
		file << " " << data->jointPositions[0][i].position.X;
		file << " " << data->jointPositions[0][i].position.Y;
		file << " " << data->jointPositions[0][i].position.Z;
	}

	file << std::endl;

	file.close();
}
