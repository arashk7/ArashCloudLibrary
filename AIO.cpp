#include "AIO.h"

namespace Arash
{
	namespace IO
	{

		AIO::AIO()
		{
		}


		AIO::~AIO()
		{
		}

		void AIO::LoadPCD(std::string file_name)
		{

		}

		int AIO::LoadPLY(std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
		{
			if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_name, *output) == -1) //* load the file
			{
				PCL_ERROR("Couldn't read file pointcloud.ply \n");
				return -1;
			}
			return 0;
		}
	}
}