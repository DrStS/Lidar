/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	// TODO: Fill in this function
	for (int i = 0; i < maxIterations; i++)
	{
		//Rand pick two points
		std::unordered_set<int> inliers;
		while (inliers.size() < 2)
			inliers.insert(rand() % (cloud->points.size()));
		//compute line from random samples
		auto iter = inliers.begin();
		float x1 = cloud->points[*iter].x;
		float y1 = cloud->points[*iter].y;
		iter++;
		float x2 = cloud->points[*iter].x;
		float y2 = cloud->points[*iter].y;
		float a = y1 - y2;
		float b = x2 - x1;
		float c = x1 * y2 - x2 * y1;

		if (cloud->points.size() > 0)
		{
			float x, y, d;
			for (size_t index = 0; index < cloud->points.size(); index++)
			{
				//compute distance
				x = cloud->points[index].x;
				y = cloud->points[index].y;
				d = fabs(a * x + b * y + c) / sqrt(a * a + b * b);
				//check for min distance
				if (d < distanceTol)
				{
					inliers.insert(index);
				}
			}
			if (inliersResult.size() < inliers.size())
			{
				inliersResult = inliers;
			}
		}
	}

	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	for (int i = 0; i < maxIterations; i++)
	{
		//Rand pick three points
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand() % (cloud->points.size()));
		//compute line from random samples
		auto iter = inliers.begin();
		float x1 = cloud->points[*iter].x;
		float y1 = cloud->points[*iter].y;
		float z1 = cloud->points[*iter].z;
		iter++;
		float x2 = cloud->points[*iter].x;
		float y2 = cloud->points[*iter].y;
		float z2 = cloud->points[*iter].z;
		iter++;
		float x3 = cloud->points[*iter].x;
		float y3 = cloud->points[*iter].y;
		float z3 = cloud->points[*iter].z;

		float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		float d = -(a * x1 + b * y1 + c * z1);

		if (cloud->points.size() > 0)
		{
			float x, y, z, dist;
			for (size_t index = 0; index < cloud->points.size(); index++)
			{
				//compute distance
				x = cloud->points[index].x;
				y = cloud->points[index].y;
				z = cloud->points[index].z;
				dist = fabs(a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c);
				//check for min distance
				if (dist < distanceTol)
				{
					inliers.insert(index);
				}
			}
			//cout << "DEBUG: inliersResult.size()"<< inliersResult.size()<<endl;
			if (inliersResult.size() < inliers.size())
			{
				inliersResult = inliers;
			}
		}
	}
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " microseconds" << std::endl;

	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 1000, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
