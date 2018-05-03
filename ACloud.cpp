#include "ACloud.h"

namespace Arash
{
	namespace Cloud
	{

		ACloud::ACloud()
		{
			cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
			normal= pcl::PointCloud<PointNT>::Ptr(new pcl::PointCloud<PointNT>);;
		}


		ACloud::~ACloud()
		{

			Release();
		}

		ACloud *ACloud::Clone()
		{
			ACloud *c = new ACloud();
			pcl::copyPointCloud(*this->cloud,*c->cloud);
			return c;
		}
		void ACloud::Release()
		{
			cloud->~PointCloud();
		}
		int ACloud::LoadPCD(std::string file_name)
		{
			if (pcl::io::loadPCDFile<PointType>(file_name, *cloud) == -1) //* load the file
			{
				PCL_ERROR("Couldn't read file pointcloud.ply \n");
				exit(0);
				return -1;
			}
			if (pcl::io::loadPCDFile<PointNT>(file_name, *normal) == -1) //* load the file
			{
				PCL_ERROR("Couldn't read normals \n");
				return -1;
			}
			return 0;
		}
		int ACloud::LoadPLY(std::string file_name)
		{
			if (pcl::io::loadPLYFile<PointType>(file_name, *cloud) == -1) //* load the file
			{
				PCL_ERROR("Couldn't read file pointcloud.ply \n");
				exit(0);
				return -1;
			}
			if (pcl::io::loadPLYFile<PointNT>(file_name, *normal) == -1) //* load the file
			{
				//PCL_ERROR("Couldn't read normals \n");
				return -1;
			}
			return 0;
		}
		
		void ACloud::Merge(ACloud *input_cloud)
		{
		
			*cloud += *input_cloud->cloud;
			*normal += *input_cloud->normal;
		
		}

		void ACloud::Translate(float x, float y, float z)
		{
			Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

			transform_2.translation() << x, y, z;

			// The same rotation matrix as before; theta radians around Z axis
			//transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
			//transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
			//transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

			pcl::transformPointCloud(*cloud, *cloud, transform_2);
		}
		void ACloud::Rotate(float t_x, float t_y, float t_z)
		{
			Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

			//transform_2.translation() << x, y, z;

			// The same rotation matrix as before; theta radians around Z axis
			transform_2.rotate(Eigen::AngleAxisf(t_x, Eigen::Vector3f::UnitX()));
			transform_2.rotate(Eigen::AngleAxisf(t_y, Eigen::Vector3f::UnitY()));
			transform_2.rotate(Eigen::AngleAxisf(t_z, Eigen::Vector3f::UnitZ()));

			pcl::transformPointCloud(*cloud, *cloud, transform_2);
		}
	}
}
