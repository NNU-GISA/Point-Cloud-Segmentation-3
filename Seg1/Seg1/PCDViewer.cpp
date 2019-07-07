
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/don.h>
using namespace pcl;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointNormal PointOutT;
typedef pcl::search::Search<PointT>::Ptr SearchPtr;


void Show2Cloud(PointCloud<PointNormal>::Ptr cloud);
void ShowACloud(PointCloud<PointT>::Ptr cloud);

int
main(int argc, char** argv)
{
	pcl::PointCloud <PointT>::Ptr cloud(new pcl::PointCloud <PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile <PointT>("5.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);

	sor.setLeafSize(0.003f, 0.003f, 0.003f);
	sor.filter(*cloud_filtered);
	//cloud = cloud_filtered;
	ShowACloud(cloud);
}

void ShowACloud(PointCloud<PointT>::Ptr cloud)
{
	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");

	viewer.setBackgroundColor(150, 150, 150, 0);

	viewer.addPointCloud(cloud);
	viewer.spin();
	while (!viewer.wasStopped())
	{
	}
}