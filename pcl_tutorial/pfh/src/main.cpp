#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/search/organized.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

void simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, \
		pcl::visualization::PCLVisualizer::Ptr viewer)
{
  //Init 3D viewer
  viewer->setBackgroundColor (0, 0, 0.5);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  //viewer->addCoordinateSystem (1.0);
  //viewer->initCameraParameters ();
}

void setNormal1 (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, \
		pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(cloud);
  ne.compute(*normals);
}

void setNormal2 (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, \
		pcl::PointCloud<pcl::Normal>::Ptr normals, \
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (1);
  ne.compute (*normals);
}

void viewPoint (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, \
		pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  simpleVis (cloud, viewer);
  if (!cloud->isOrganized ()){
	 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	 setNormal2 (cloud, normals, tree);}
  else{
	 setNormal1 (cloud, normals);}
  viewer->setBackgroundColor (0.0, 0.0, 0.5);
  viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce();
  }
}

void generateCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, \
		std::int32_t width)
{
	cloud->width  = width;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
	  cloud->points[i].x = 2 + 1024 * rand () / (RAND_MAX + 1.0f);
	  cloud->points[i].y = 2 + 1024 * rand () / (RAND_MAX + 1.0f);
	  cloud->points[i].z = 2 + 1024 * rand () / (RAND_MAX + 1.0f) / 100;
	}
}

void coputePfh (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,\
		pcl::PointCloud<pcl::Normal>::Ptr normals,\
		pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh,\
		pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	if (!cloud->isOrganized ()){
	  setNormal2 (cloud, normals,tree);
	  pfh.setSearchMethod (tree);
	}
	else{
	  setNormal1 (cloud, normals);
	  //pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr neighbor \
	  (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
	  pfh.setSearchMethod (tree);
	}
    pfh.setInputCloud (cloud);
	pfh.setInputNormals (normals);
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	pfh.setRadiusSearch (1.5);
	// Compute the features
	pfh.compute (*pfhs);
}

int main (int argc, char** argv){
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   //pcl::io::loadPCDFile ("/home/wuhang/pclfile/table_scene_mug_stereo_textured.pcd", *cloud);
   generateCloud (cloud, 500);
   pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

   // Create the PFH estimation class, and pass the input dataset+normals to it
   pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
   // Output datasets
   pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

   coputePfh(cloud,normals,pfh,pfhs);
   pcl::visualization::PCLPlotter plotter;
   plotter.addFeatureHistogram<pcl::PFHSignature125> (*pfhs,"pfh",100);
   //viewPoint(cloud, normals);
   plotter.plot();
   cout<<"done"<<endl;
   return (0);
}

