// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
// #include "quiz/cluster/kdtree.h"
//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr croppedCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filteredCloud);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*croppedCloud);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true); //remove roofpoints where Lidar hits the roof
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(croppedCloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int point: indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(croppedCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*croppedCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return croppedCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr roadCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); //set to false to extract inliers
    extract.filter(*roadCloud);
    extract.setNegative(true); //set to true to extract outliers
    extract.filter(*obstacleCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    //returns segmented obstacle and road clouds
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  for (int j = 0; j < maxIterations; ++j)
  {
    std::unordered_set<int> inliersTemp;
    PointT point1, point2, point3;
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
      PointT point4 = cloud->points[i];
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

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloudOutliers, cloudInliers);
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j(0);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>());

        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit!=it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
        j++;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(int indice, typename pcl::PointCloud<PointT>::Ptr& cloud, std::vector<int>& cluster, std::vector<bool>& pointMark, KdTree<PointT>* tree, float distanceTol)
{

  pointMark[indice] = 1;
  cluster.push_back(indice);
  std::vector<int> nearbyPoints = tree->search(cloud->points[indice], distanceTol); //returns list of indices that are nearby indice
  for (int i = 0; i < nearbyPoints.size(); ++i)
  {
    if(!pointMark[nearbyPoints[i]])
      Proximity(nearbyPoints[i], cloud, cluster, pointMark, tree, distanceTol);
  }

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize)
{
    KdTree<PointT>* tree = new KdTree<PointT>();

    for (int i = 0; i < cloud->points.size(); ++i)
    {
        tree->insert(cloud->points[i], i);
    }

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> pointMark(cloud->points.size(), 0);
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        if (!pointMark[i])
        {
          std::vector<int> cluster_idx;
          typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
          Proximity(i, cloud, cluster_idx, pointMark, tree, clusterTolerance);

          if(cluster_idx.size() >= minSize && cluster_idx.size() <= maxSize)
          {
            for (int i = 0; i < cluster_idx.size(); ++i)
            {
                cluster->points.push_back(cloud->points[cluster_idx[i]]);
            }

            cluster->width = cluster->points.size();
            cluster->height = 1;
            clusters.push_back(cluster);
          }

          else
          {
            for (int i = 0; i < cluster_idx.size(); ++i)
            {
                pointMark[cluster_idx[i]] = 0;
            }
          }
          
        }
    }

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

// template<typename PointT>
// BoxQ ProcessPointClouds<PointT>::BoundingBoxPCA(typename pcl::PointCloud<PointT>::Ptr cluster)
// {
//     Eigen::Vector4f centroid;
//     pcl::compute3DCentroid(*cluster, centroid);
//     Eigen::Matrix3f covariance;
//     pcl::computeCovarianceMatrixNormalized(*cluster, centroid, covariance);
//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
//     Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
//     eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

//     Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
//     p2w.block<3,3>(0,0) = eigDx.transpose();
//     p2w.block<3,3>(0,3) = -1.f*(p2w.block<3,3>(0, 0)*centroid.head<3>());
//     pcl::PointCloud<PointT> cPoints;
//     pcl::transformPointCloud(*cluster, cPoints, p2w);

//     PointT min_pt, max_pt;
//     pcl::getMinMax3D(cPoints, min_pt, max_pt);
//     const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

//     BoxQ boxq;
//     boxq.bboxTransform = eigDx*mean_diag + centroid.head<3>();
//     boxq.bboxQuaternion = eigDx;
//     boxq.cube_length = max_pt.x - min_pt.x;
//     boxq.cube_width = max_pt.y - min_pt.y;
//     boxq.cube_height = max_pt.z - min_pt.z;

//     return boxq;
// }


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}