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
//#include "AIO.h"

namespace Arash
{
	namespace Cloud
	{

		class ACloud 
		{
		public:
			ACloud();
			~ACloud();
			typedef pcl::PointXYZ PointType;
			typedef pcl::PointNormal PointNT;

			pcl::PointCloud<PointType>::Ptr cloud;
			pcl::PointCloud<PointNT>::Ptr normal;
			ACloud *Clone();
			void Release();
			int LoadPCD(std::string file_name);
			int LoadPLY(std::string file_name);
			
			void Merge(ACloud  *input_cloud);
			void Translate(float x, float y, float z);
			void Rotate(float t_x, float t_y, float t_z);

		};
	}
}

