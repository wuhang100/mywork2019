#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Point cloud input
  cloud->width  = 10;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 2 + 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 2 + 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 2 + 1024 * rand () / (RAND_MAX + 1.0f) / 10;
  }

  std::cout << "Cloud input: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (2);

  // Compute the features
  ne.compute (*cloud_normals);
  cloud_normals->points.size ();
  std::cout << "Cloud normals: " << std::endl;
  for (size_t i = 0; i < cloud_normals->points.size (); ++i)
    std::cout << "    " << cloud_normals->points[i].normal_x << " "
                        << cloud_normals->points[i].normal_y << " "
                        << cloud_normals->points[i].normal_z << std::endl;

  return (0);
}
