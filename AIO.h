#pragma once
#define NOMINMAX
#include <Windows.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/filters/voxel_grid.h>

namespace Arash
{
	namespace IO
	{
		class AIO
		{
		public:
			
			int LoadPCD(std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
			int LoadPLY(std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
		};

	}
}