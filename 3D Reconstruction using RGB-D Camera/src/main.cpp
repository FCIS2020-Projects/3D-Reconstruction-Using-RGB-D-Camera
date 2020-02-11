#include "InputHelper.h"
#include "rigestration.h"

int main()
{
	cout << "////////////////////// PIXELZ /////////////////////" << endl;
	cout << "///////////// 3D Reconstruction System ////////////\n" << endl <<endl;

	cout << "1 -> Capture data and PCD Converting (Only Arsany)" << endl; 
	cout << "2 -> Rigestration Model To build Point Cloud" << endl;

	int choice = 0; 

	cin >> choice;
	if (choice == 1) {

		cout << "Starting Capturing Data!" << endl;
		InputHelper ih(640, 480, 60);
		ih.getCameraInput();
	}
	else if (choice == 2) {
		cout << "Start Rigestration!" << endl;
		std::string folderName, fileName;
		int num_of_pcd;
		rigestration ri;

		cout << "Please Enter the Folder Name in Debug = ";
		cin >> folderName;

		cout << "Please Enter the File Name in Debug = ";
		cin >> fileName;

		cout << "Please Enter the # of pcds (1 -> 99) = ";
		cin >> num_of_pcd;

		if (folderName == "NA" || fileName == "NA" || num_of_pcd <= 0 || num_of_pcd >99)
		{
			folderName = "dog_pcd_data";
			fileName = "dog_day_";
			num_of_pcd = 20;
		}


		ri.runRigestration(folderName, fileName, num_of_pcd);
	}
	else{
		cout << "Error Parameter!\n Please Run again with a Valid input" << endl;
		return 0;
	}
	
	//ih.visualizePCD("arsany01.pcd");
	return 0;
}