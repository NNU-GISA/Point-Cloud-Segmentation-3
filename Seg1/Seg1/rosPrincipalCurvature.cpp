#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
ros::Publisher pub;
float leafsize = 0.004f;
//float normalsearchradius = 0.01f;
int Ksearch = 12;
float pcsearchradius = 0.025f;

float
interpolate(float value, float min, float max) {
	return (value - min) / (max - min);
}

int
HeatMapGetRed(float value, float max) {
	float ret;
	float r = value / max;
	if (r < 0.5)
		ret = 0;
	else if (r > 0.75)
		ret = 1;
	else
		ret = interpolate(r, 0.5, 0.75);

	return ret * 255;
}

int
HeatMapGetGreen(float value, float max) {
	float ret;
	float r = value / max - 0.5;
	if (r < 0)
		r = -r;

	if (r > 0.25)
		ret = interpolate(r, 0.5, 0.25);
	else
		ret = 1;

	return ret * 255;
}

int
HeatMapGetBlue(float value, float max) {
	float ret;
	float r = value / max;
	if (r > 0.5)
		ret = 0;
	else if (r < 0.25)
		ret = 1;
	else
		ret = interpolate(r, 0.25, 0.5);

	return ret * 255;
}

float
GraspQuality1(pcl::Normal norm, pcl::PrincipalCurvatures pc) {
	float retval = 0;
	retval += pc.pc1;// -pc.pc2*pc.pc2;
	return retval;
}

float
GraspQuality2(pcl::Normal norm, pcl::PrincipalCurvatures pc) {
	float retval = 0;
	retval += pc.pc1 - pc.pc2;
	return retval;
}

void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Create a container for the data.
	sensor_msgs::PointCloud2 output;

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*input, cloud);

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(incloud);

	sor.setLeafSize(leafsize, leafsize, leafsize);
	sor.filter(*cloud);

	// Compute the normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::Normal>);

	//normal_estimation.setRadiusSearch(normalsearchradius);
	normal_estimation.setKSearch(Ksearch);
	normal_estimation.compute(*cloud_with_normals);

	// Setup the principal curvatures computation
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

	// Provide the original point cloud (without normals)
	principal_curvatures_estimation.setInputCloud(cloud);

	// Provide the point cloud with normals
	principal_curvatures_estimation.setInputNormals(cloud_with_normals);

	// Use the same KdTree from the normal estimation
	principal_curvatures_estimation.setSearchMethod(tree);
	principal_curvatures_estimation.setRadiusSearch(pcsearchradius);
	//principal_curvatures_estimation.setKSearch(60);

	// Actually compute the principal curvatures
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>());
	principal_curvatures_estimation.compute(*principal_curvatures);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	// Fill in the cloud data
	colored_cloud->width = cloud->width;
	colored_cloud->height = cloud->height;
	colored_cloud->is_dense = false;
	colored_cloud->points.resize(colored_cloud->width * colored_cloud->height);
	float minDataVal = 0;
	float maxDataValue = 0;
	int indexOfMax = 0;

	float* graspQuality = new float[cloud->points.size()];

	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		graspQuality[i] = GraspQuality2(cloud_with_normals->points[i], principal_curvatures->points[i]);

		if (graspQuality[i] > maxDataValue) {
			maxDataValue = graspQuality[i];
			indexOfMax = i;
		}
	}

	for (size_t i = 0; i < colored_cloud->points.size(); ++i)
	{
		colored_cloud->points[i].x = cloud->points[i].x;
		colored_cloud->points[i].y = cloud->points[i].y;
		colored_cloud->points[i].z = cloud->points[i].z;
		float value = graspQuality[i];
		colored_cloud->points[i].r = HeatMapGetRed(value, maxDataValue);
		colored_cloud->points[i].g = HeatMapGetGreen(value, maxDataValue);
		colored_cloud->points[i].b = HeatMapGetBlue(value, maxDataValue);
	}

	pcl::toROSMsg(*output, colored_cloud);

	// Publish the data.
	pub.publish(output);
}

int
main(int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "principalcurvature");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

	// Spin
	ros::spin();
}