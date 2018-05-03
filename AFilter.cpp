#include "AFilter.h"

namespace Arash
{
	namespace Filter
	{
		

		AFilter::AFilter()
		{
		}


		AFilter::~AFilter()
		{
		}

		void AFilter::VoxelGrid(ACloud *source, ACloud *dest, float param)
		{
			
			pcl::VoxelGrid<pcl::PointXYZ> sor;
			sor.setInputCloud(source->cloud);
			sor.setLeafSize(param, param, param);
			sor.filter(*dest->cloud);

			pcl::VoxelGrid<pcl::PointNormal> sor1;
			sor1.setInputCloud(source->normal);
			sor1.setLeafSize(param, param, param);
			sor1.filter(*dest->normal);
		}
		void AFilter::Match(ACloud *cloud1, ACloud *cloud2, ACloud *dest)
		{
			const float leaf = 0.005f;
			// Types
			typedef pcl::PointNormal PointNT;
			typedef pcl::PointCloud<PointNT> PointCloudT;
			typedef pcl::FPFHSignature33 FeatureT;
			typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
			typedef pcl::PointCloud<FeatureT> FeatureCloudT;

			// Point clouds
			PointCloudT::Ptr object(new PointCloudT);
			PointCloudT::Ptr object_aligned(new PointCloudT);
			PointCloudT::Ptr scene(new PointCloudT);
			FeatureCloudT::Ptr object_features(new FeatureCloudT);
			FeatureCloudT::Ptr scene_features(new FeatureCloudT);

			scene = cloud2->normal;
			object = cloud1->normal;

			// Estimate normals for scene
			pcl::console::print_highlight("Estimating scene normals...\n");
			pcl::NormalEstimationOMP<PointNT, PointNT> nest;
			nest.setRadiusSearch(0.01);
			nest.setInputCloud(scene);
			nest.compute(*scene);

			// Estimate features
			pcl::console::print_highlight("Estimating features...\n");
			FeatureEstimationT fest;
			fest.setRadiusSearch(0.025);
			fest.setInputCloud(object);
			fest.setInputNormals(object);
			fest.compute(*object_features);
			fest.setInputCloud(scene);
			fest.setInputNormals(scene);
			fest.compute(*scene_features);

			// Perform alignment
			pcl::console::print_highlight("Starting alignment...\n");
			pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
			align.setInputSource(object);
			align.setSourceFeatures(object_features);
			align.setInputTarget(scene);
			align.setTargetFeatures(scene_features);
			align.setMaximumIterations(100000); // Number of RANSAC iterations //50000
			align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
			align.setCorrespondenceRandomness(5); // Number of nearest features to use //5
			align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
			align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
			align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
			{
				pcl::ScopeTime t("Alignment");
				align.align(*object_aligned);
			}

			dest->normal = object_aligned;
			if (align.hasConverged())
			{
				// Print results
				printf("\n");
				Eigen::Matrix4f transformation = align.getFinalTransformation();
				pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
				pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
				pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
				pcl::console::print_info("\n");
				pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
				pcl::console::print_info("\n");
				pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());
			}
		}
	}
}