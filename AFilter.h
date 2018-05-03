#pragma once
#define NOMINMAX
#include <Windows.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include "ACloud.h"

using namespace Arash::Cloud;

namespace Arash
{
	namespace Filter
	{
		class AFilter
		{
		public:
			

			
		public:
			AFilter();
			~AFilter();
			void VoxelGrid(ACloud *source, ACloud *dest, float param);
			void Match(ACloud *cloud1, ACloud *cloud2, ACloud *dest);
			
		};

	}
}