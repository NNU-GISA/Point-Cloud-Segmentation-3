
#include <iostream>
#include <vector>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ctime>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
//boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(pcl::PointCloud<pcl::PointNormal>::Ptr normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
//boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr normals);



clock_t begin1 = clock();
void PrintTime(char* str) {
	using namespace std;

	clock_t now = clock();
	double elapsed_secs = double(now - begin1) / CLOCKS_PER_SEC;
	std::cout << str << elapsed_secs << endl;
}

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

int
HeatMapGetRedOld(float value, float max) {
	if (value < 0)
		return 0;


	float v = value / max * 255;

	if (v > 255)
		v = 255;

	return v;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1)
{
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->addCoordinateSystem(1.0);

	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);

	return (viewer);
}

int
HeatMapGetGreenOld(float value, float max) {
	float v = (max - value)*255;

	if (v < 0)
		v = 0;

	return v;
}

int
HeatMapGetBlueOld(float value, float max) {
	if (value > 0)
		return 0;
	
	float v = -value / max * 255;

	if (v > 255)
		v = 255;

	return v;
}

float 
GraspQuality1(pcl::Normal norm, pcl::PrincipalCurvatures pc){
	float retval = 0;
	//retval += norm.normal_z / 35;
	retval += pc.pc1;// -pc.pc2*pc.pc2;
	return retval;
}

float
GraspQuality2(pcl::Normal norm, pcl::PrincipalCurvatures pc) {
	float retval = 0;
	//retval += norm.normal_z / 35;
	retval += pc.pc1 - pc.pc2;
	return retval;
}




int
main(int, char** argv)
{
	std::string filename = "4.pcd";
	std::cout << "Reading " << filename << std::endl;
	float leafsize = 0.0035f;
	float normalsearchradius = 0.011f;
	int Ksearch = 60;
	float pcsearchradius = 0.028f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr incloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *incloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file");
		return (-1);
	}

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(incloud);

	sor.setLeafSize(leafsize, leafsize, leafsize);
	sor.filter(*cloud);
	//cloud = cloud_filtered;
	std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;
	PrintTime("Loaded  ");
	// Compute the normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::Normal>);

	//normal_estimation.setRadiusSearch(normalsearchradius);
	normal_estimation.setKSearch(Ksearch);

	normal_estimation.compute(*cloud_with_normals);
	PrintTime("norms  ");
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
	PrintTime("pcs  ");
	std::cout << "output points.size (): " << principal_curvatures->points.size() << std::endl;

	// Display and retrieve the shape context descriptor vector for the 0th point.
	//pcl::PrincipalCurvatures descriptor = principal_curvatures->points[0];
	//std::cout << descriptor << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	//pcl::PointCloud<pcl::PointXYZRGBA> colored_cloud;

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

		if(cloud_with_normals->points[i].curvature < 0)
			std::cout << "quality " << cloud_with_normals->points[i].curvature << std::endl;

		if (graspQuality[i] > maxDataValue) {
			maxDataValue = graspQuality[i];
			indexOfMax = i;
			std::cout << "quality " << graspQuality[i] << std::endl;
			std::cout << "i " << i << std::endl;
		}
	}

 	for (size_t i = 0; i < colored_cloud->points.size(); ++i)
	{
		colored_cloud->points[i].x = cloud->points[i].x;
		colored_cloud->points[i].y = cloud->points[i].y;
		colored_cloud->points[i].z = cloud->points[i].z;
		float value = graspQuality[i];//(principal_curvatures->points[i].pc1 / (maxDataValue - minDataVal));   // First you should normalize to a number between 0 and 1.
		//std::cout << "quality " << value << std::endl;
		colored_cloud->points[i].r = HeatMapGetRed(value, maxDataValue);
		colored_cloud->points[i].g = HeatMapGetGreen(value, maxDataValue);
		colored_cloud->points[i].b = HeatMapGetBlue(value, maxDataValue);
		if (i == indexOfMax) {
			std::cout << "red " << HeatMapGetGreen(value, maxDataValue) << std::endl;
			std::cout << "i " << i << std::endl;
		}
	}

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cluster viewer"));
	//viewer.setBackgroundColor(0, 0, 0, 0);
	/*viewer.showCloud(&colored_cloud);*/
	//viewer.addPointCloud(colored_cloud);
	//viewportsVis(colored_cloud, cloud_with_normals);

	int v1(0);
	//viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	//viewer->addText("Principal Curvature", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
	float scale = 0.2;
	float x = cloud_with_normals->points[indexOfMax].normal_x*scale +cloud->points[indexOfMax].x;
	float y = cloud_with_normals->points[indexOfMax].normal_y*scale +cloud->points[indexOfMax].y;
	float z = cloud_with_normals->points[indexOfMax].normal_z*scale +cloud->points[indexOfMax].z;
	
	float x1 = principal_curvatures->points[indexOfMax].principal_curvature_x*scale + cloud->points[indexOfMax].x;
	float y1 = principal_curvatures->points[indexOfMax].principal_curvature_y*scale + cloud->points[indexOfMax].y;
	float z1 = principal_curvatures->points[indexOfMax].principal_curvature_z*scale + cloud->points[indexOfMax].z;
	pcl::PointXYZ p(x,y,z);
	pcl::PointXYZ p1(x1, y1, z1);
	viewer->addLine(cloud->points[indexOfMax], p, "line1");
	viewer->addLine(cloud->points[indexOfMax], p1, "line2");
	//viewer->addLine(cloud->points[indexOfMax], p1);
	viewer->setShowFPS(true);
	viewer->setSize(1440, 900);
	viewer->setWindowBorders(true);
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> intens(cloud, "intensity");
	//viewer->addPointCloud<pcl::PointXYZINormal>(cloud, intens, "sample cloud", v1);
	viewer->addCoordinateSystem(0.2);

	//viewer->addPointCloudPrincipalCurvatures<pcl::PointXYZ, pcl::Normal>(cloud, cloud_with_normals, principal_curvatures);
	viewer->spin();
	while (!viewer->wasStopped())
	{
	}

	return 0;
}