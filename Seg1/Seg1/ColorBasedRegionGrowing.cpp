#define PCL_NO_PRECOMPILE true;

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

int
main(int argc, char** argv)
{
	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr inputcloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile <pcl::PointXYZRGB>("3.pcd", *inputcloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(inputcloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	//for (int i = 0; i < 300000;i++) {
	//	std::cout << std::to_string(inputcloud->points[i].x) << std::endl;
	//}

	//pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);

	//pcl::removeNaNFromPointCloud(*inputcloud, *cloud, *indices);

	//pcl::IndicesPtr indices2(new std::vector <int>);
	//pcl::PassThrough<pcl::PointXYZRGB> pass2;
	//pass2.setInputCloud(cloud);
	//pass2.setFilterFieldName("z");
	//pass2.setFilterLimits(0.0, 1.0);
	//pass2.filter(*indices2);

	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud(inputcloud);
	reg.setIndices(indices);
	reg.setSearchMethod(tree);
	reg.setDistanceThreshold(0.005);
	reg.setPointColorThreshold(6);
	reg.setRegionColorThreshold(5);
	reg.setMinClusterSize(600);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::PCLVisualizer viewer("Cluster viewer");
	viewer.setBackgroundColor(150, 150, 150, 0);
	//viewer.showCloud(colored_cloud);
	viewer.addPointCloud(colored_cloud);
	viewer.spin();
	while (!viewer.wasStopped())
	{
	}
	return (0);

	return (0);
}