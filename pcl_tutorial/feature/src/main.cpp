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
#include <pcl/filters/filter.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/extract_indices.h>

void generateCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, \
		            std::int32_t width, std::int32_t height )
{
	if (height == 1){
		cloud->width  = width;
		cloud->height = height;
		cloud->points.resize (cloud->width * cloud->height);
		float j = 2.0; float x_base = 2.0; float y_base = 2.0; float z_base = 2.0;
		for (std::int32_t i = 0; i < cloud->points.size (); ++i){
			if (i%10 == 0){
				x_base = 0;
				y_base = y_base + 0.1;
				z_base = z_base + 0.1;
		        }
		    cloud->points[i].x = x_base;
		    cloud->points[i].y = y_base + 0.001 * rand () / (RAND_MAX + 1.0f);
		    cloud->points[i].z = z_base + 0.01 * rand () / (RAND_MAX + 1.0f);
		    x_base = x_base + 0.1;
		}
	}

	else{
		cloud->width  = width;
		cloud->height = height;
		cloud->points.resize (cloud->width * cloud->height);
		float j = 2.0; float x_base = 2.0; float y_base = 2.0; float z_base = 2.0;
		for (std::int32_t i = 0; i < cloud->points.size (); i++){
			if (i%width == 0){
				x_base = 0;
				y_base = y_base + 0.1;
				z_base = z_base + 0.1*(i/cloud->width);
		        }
		    cloud->points[i].x = x_base;
		    cloud->points[i].y = y_base;
		    cloud->points[i].z = z_base;
		    x_base = x_base + 0.1;
		/*	    cloud->points[i].x = 2 + 1024 * rand () / (RAND_MAX + 1.0f);
			    cloud->points[i].y = 2 + 1024 * rand () / (RAND_MAX + 1.0f);
			    cloud->points[i].z = 2 + 1024 * rand () / (RAND_MAX + 1.0f) / 100;*/
			}
	}
}

void simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, \
		pcl::visualization::PCLVisualizer::Ptr viewer){
    //Init 3D viewer
    viewer->setBackgroundColor (0, 0, 0.5);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem (1.0);
    //viewer->initCameraParameters ();
}

void setNormal1 (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, \
		pcl::PointCloud<pcl::Normal>::Ptr normals){
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    //ne.setRectSize(5,5);
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(1.0f);
    ne.setNormalSmoothingSize(5.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
}

void setNormal2 (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, \
		pcl::PointCloud<pcl::Normal>::Ptr normals, \
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree){
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod (tree);
    //ne.setIndices();
    ne.setRadiusSearch (1);
    ne.compute (*normals);
}

void setindices (std::int32_t search_point, std::vector<int>& search_indices,\
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,\
		std::int32_t search_width, std::int32_t search_height){
	search_indices.push_back(search_point);
	std::int32_t i_x = search_point / cloud->width;
	std::int32_t i_y = search_point % cloud->width;
	//std::cout << "i_x is " << i_x <<" i_y is " << i_y << std::endl;
	std::int32_t i_n;
	for (std::int32_t j = i_x - search_height/2; j <= i_x + search_height/2; j++){
		for (std::int32_t k = i_y - search_width/2; k <= i_y + search_width/2; k++){
			//std::cout << "search point " << j <<" " << k << " ";
			if (j>=0 && j<cloud->height && k>=0 && k<cloud->width &&!(j==i_x && k==i_y)){
			    search_indices.push_back(j* cloud->width + k);
			    //std::cout << "valid " << j* cloud->width + k;
			}
			//std::cout << std::endl;
		}
	}
	//for (std::int32_t i = 0; i < search_indices.size(); i++){
	        //std::cout << search_indices[i] << std::endl;
	//}
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

void viewPoint (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, \
		pcl::PointCloud<pcl::Normal>::Ptr normals){
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    simpleVis (cloud, viewer);
    viewer->setBackgroundColor (0.0, 0.0, 0.5);
    //viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
    while (!viewer->wasStopped ()){
    viewer->spinOnce();
    }
    if (!cloud->isOrganized ()){
    	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    	setNormal2 (cloud, normals, tree);}
    else{
    	setNormal1 (cloud, normals);}
    //viewer->setBackgroundColor (0.0, 0.0, 0.5);
    //viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
    //while (!viewer->wasStopped ()){
    //viewer->spinOnce();
    //}
}

void setNormal3 (boost::shared_ptr<std::vector<int>> pointnum_ptr, \
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_f, \
		pcl::PointCloud<pcl::Normal>::Ptr normals, \
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree){
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_f);
    ne.setSearchMethod (tree);
    //ne.setIndices();
    ne.setRadiusSearch (5);
    ne.setIndices (pointnum_ptr);
    ne.compute (*normals);
}

void viewnormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, \
		pcl::PointCloud<pcl::Normal>::Ptr normals,\
		std::int32_t search_width, std::int32_t search_height,\
		std::vector<int> search_pointnum,\
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,\
		pcl::PointCloud<pcl::Normal>::Ptr pointnormal,\
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f){
	//std::vector<int> indices;
	std::int32_t search_point;
	for (std::int32_t i = 0; i < normals->points.size (); i++)
	{
    if (!pcl_isnan(normals->points[i].normal_x)){

    }
    else{
    	search_point = i;
    	pcl::PointCloud<pcl::PointXYZ> cloud_fi;
    	std::vector<int> search_indices;
    	setindices (search_point, search_indices,cloud,search_width,search_height);
    	boost::shared_ptr<std::vector<int>> indices_ptr = boost::make_shared<std::vector<int>>(search_indices);
    	boost::shared_ptr<std::vector<int>> pointnum_ptr = boost::make_shared<std::vector<int>>(search_pointnum);
    	pcl::ExtractIndices<pcl::PointXYZ> extract;
    	extract.setInputCloud(cloud);
    	extract.setIndices (indices_ptr);
    	extract.filter (cloud_fi);
    	cloud_f=cloud_fi.makeShared();
    	setNormal3 (pointnum_ptr, cloud_f, pointnormal, tree);
    	normals->points[i].normal_x = pointnormal->points[0].normal_x;
    	normals->points[i].normal_y = pointnormal->points[0].normal_y;
    	normals->points[i].normal_z = pointnormal->points[0].normal_z;
/*    	std::cout << " ( " << i / cloud->width + 1 << " , "<< i % cloud->width + 1 << "): "
              << " Points: (" << cloud->points[i].x << ","
    		  << cloud->points[i].y << ","
    		  << cloud->points[i].z << ")"
    		  <<"  Normals: (" << pointnormal->points[0].normal_x << ","
    		  << pointnormal->points[0].normal_y << ","
    		  << pointnormal->points[0].normal_z << ")"
              << std::endl;*/
    }
	std::cout << " ( " << i / cloud->width + 1 << " , "<< i % cloud->width + 1 << "): "
          << " Points: (" << cloud->points[i].x << ","
		  << cloud->points[i].y << ","
		  << cloud->points[i].z << ")"
		  <<"  Normals: (" << normals->points[i].normal_x << ","
		  << normals->points[i].normal_y << ","
		  << normals->points[i].normal_z << ")"
          << std::endl;
	}

}


int main (){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr pointnormal (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	std::vector<int> search_pointnum {0};
	std::int32_t width, height, search_width, search_height;


	std::cout<< "Enter the dataset size (width, height)" <<std::endl;
	std::cin>>width>>height;
	generateCloud (cloud, width, height);
	//std::vector<int> mapping;
	std::cout<< "Enter the search region size (width, height)" <<std::endl;
	std::cin>>search_width>>search_height;

	viewPoint(cloud, normals);
	viewnormals(cloud, normals, search_width, search_height, search_pointnum, tree, pointnormal, cloud_f);
    return(0);
    //cloud_f->points.resize (cloud_f->width * cloud_f->height);

}
