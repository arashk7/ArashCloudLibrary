
#define NOMINMAX
//#define _CRT_SECURE_NO_WARNINGS

#include <Windows.h>
//
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/surface/poisson.h>
//#include <pcl/surface/texture_mapping.h>
//#include <pcl/filters/voxel_grid.h>

///opencv
//#include <opencv2/opencv.hpp>
#include <opencv\highgui.h>
#include "ACloud.h"
#include "AFilter.h"

using namespace Arash::Cloud;
using namespace Arash::Filter;

typedef pcl::PointXYZ PointType;
typedef pcl::PointNormal PointNT;
int main(void)
{
		
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
		new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setCameraPosition(2.0, 1.0, -1.5,180.0, 0.0, 0.0);

	AFilter *aFilter = new AFilter();
	
	
	
	ACloud *aCloud11 = new ACloud();
	aCloud11->LoadPCD("chef.pcd");
	
	ACloud *aCloud22 = new ACloud();
	aCloud22->LoadPCD("rs1.pcd");

	aFilter->VoxelGrid(aCloud11, aCloud11, 0.01f);
	aFilter->VoxelGrid(aCloud22, aCloud22, 0.01f);

	aFilter->Match(aCloud11, aCloud22, aCloud11);
	aCloud11->Merge(aCloud22);
	
	viewer->addPointCloud<PointNT>(aCloud11->normal, "cloud2");
	/// Load different part of the point clouds which have a common points for matching
	
	ACloud *aCloud1 = new ACloud();
	aCloud1->LoadPLY("head_1.ply");

	ACloud *aCloud2 = new ACloud();
	aCloud2->LoadPLY("head_2.ply");

	ACloud *aCloud3 = new ACloud();
	aCloud3->LoadPLY("head_3.ply");

	ACloud *aCloud4 = new ACloud();
	aCloud4->LoadPLY("head_4.ply");

	
	
	

	aFilter->VoxelGrid(aCloud1, aCloud1, 0.01f);
	aFilter->VoxelGrid(aCloud2, aCloud2, 0.01f);
	aFilter->VoxelGrid(aCloud3, aCloud3, 0.01f);
	aFilter->VoxelGrid(aCloud4, aCloud4, 0.01f);

	
	

	aFilter->Match(aCloud1, aCloud3, aCloud1); //match the first and third pointcloud which have common points
	aCloud1->Merge(aCloud3); //merge them together

	aFilter->Match(aCloud2, aCloud4, aCloud2); //match the second and fourth pointcloud which have common points
	aCloud2->Merge(aCloud4); //merge them together

	aFilter->Match(aCloud1, aCloud2, aCloud1); //match the result of last two steps
	aCloud1->Merge(aCloud2); //reconstruct the final result
	
	//aCloud1->Translate(1, 0, 0); //translate if necessary 
	aCloud1->Rotate(90, 0, 0); //rotate if necessary 
	

	cvNamedWindow("win1", CV_WINDOW_AUTOSIZE);
	int trackbarVal = 25;
	int preVal = trackbarVal;
	int maxVal = 20;
	cvCreateTrackbar("bar1", "win1", &trackbarVal, maxVal);  //define the window and trackbar to control the voxel grid size

	ACloud *acl = aCloud1->Clone(); 
	
	while (!viewer->wasStopped())
	{
		int key;
		key = cvWaitKey(1);

		aCloud1 = acl->Clone();

		// Update Viewer
		if (trackbarVal == 0)
		{
			viewer->removePointCloud("cloud1");
			viewer->addPointCloud(aCloud1->cloud, "cloud1");

		}
		else if (preVal != trackbarVal)
		{
			float param = (float)trackbarVal / 1000.0f;
			aFilter->VoxelGrid(aCloud1, aCloud1, param);

			
			viewer->removePointCloud("cloud1");

			viewer->addPointCloud(aCloud1->cloud, "cloud1");

			std::cerr << "PointCloud after filtering: " << aCloud1->cloud->width * aCloud1->cloud->height << endl;
		}
		aCloud1->Release();

		preVal = trackbarVal;
		viewer->spinOnce();
	}

	return (0);
}
