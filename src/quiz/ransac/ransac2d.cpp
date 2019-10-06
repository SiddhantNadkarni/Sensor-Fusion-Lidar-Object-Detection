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
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
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
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
  float dist;
  for (int j = 0; j < maxIterations; ++j)
  {
    std::unordered_set<int> inliersTemp;
    pcl::PointXYZ point1, point2;
    int firstRand = rand()%(cloud->points.size());
    int secRand = rand()%(cloud->points.size());
    inliersTemp.insert(firstRand);
    inliersTemp.insert(secRand);
    point1 = cloud->points[firstRand];
    float x1 = cloud->points[firstRand].x;
    float y1 = cloud->points[firstRand].y;


    point2 = cloud->points[secRand];
    float x2 = cloud->points[secRand].x;
    float y2 = cloud->points[secRand].y;


    for (int i = 0; i < cloud->points.size(); ++i)
    {
      if(inliersTemp.count(i)>0)
        continue;
      pcl::PointXYZ point3;
      point3 = cloud->points[i];
      float x0 = point3.x;
      float y0 = point3.y;
      dist = fabs((y2 - y1)*x0 - (x2 - x1)*y0 + x2*y1 - y2*x1)/(sqrt((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1)));
      if(dist <= distanceTol)
        inliersTemp.insert(i);
    }
    if(inliersTemp.size() > inliersResult.size())
      inliersResult = inliersTemp;

    
  }



	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  for (int j = 0; j < maxIterations; ++j)
  {
    std::unordered_set<int> inliersTemp;
    pcl::PointXYZ point1, point2, point3;
    int firstRand = rand()%(cloud->points.size());
    int secRand = rand()%(cloud->points.size());
    int thirdRand = rand()%(cloud->points.size());

    inliersTemp.insert(firstRand);
    inliersTemp.insert(secRand);
    inliersTemp.insert(thirdRand);

    point1 = cloud->points[firstRand];
    float x1 = point1.x;
    float y1 = point1.y;
    float z1 = point1.z;


    point2 = cloud->points[secRand];
    float x2 = point2.x;
    float y2 = point2.y;
    float z2 = point2.z;

    point3 = cloud->points[thirdRand];
    float x3 = point3.x;
    float y3 = point3.y;
    float z3 = point3.z;

    float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);

    float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float D = -(A*x1 + B*y1 + C*z1);


    for (int i = 0; i < cloud->points.size(); ++i)
    {
      if(inliersTemp.count(i)>0)
        continue;
      pcl::PointXYZ point4 = cloud->points[i];
      float x0 = point4.x;
      float y0 = point4.y;
      float z0 = point4.z;

      float dist = fabs(A*x0 + B*y0 + C*z0 + D)/(sqrt(A*A + B*B + C*C));
      if(dist <= distanceTol)
        inliersTemp.insert(i);
    }

    if(inliersTemp.size() > inliersResult.size())
      inliersResult = inliersTemp;

  }



  return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  	renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
