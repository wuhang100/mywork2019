#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>

using namespace std;
using namespace pcl;

template <class Type>
Type stringToNum(const string& str){
	istringstream iss(str);
	Type num;
	iss >> num;
	return num;
}

void get2pcl (string file_dir, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	ifstream inFile(file_dir, ios::in);
	string lineStr;
	float xyz;
	int line_index;
	vector<vector<float>> strArray;
	while (getline(inFile, lineStr)){
		line_index = 1;
		stringstream ss(lineStr);
		string str;
		vector<float> lineArray;
		while (getline(ss, str, ',')){
		    if (line_index >= 2 && line_index <= 4){
		        xyz = stringToNum<float>(str);
		        lineArray.push_back(xyz);
		    }
		    line_index++;
		}
		strArray.push_back(lineArray);
	}
	//cout << strArray[0][0] << endl;
	cout << strArray.size() << endl;

	cloud->width = strArray.size();
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);
	for (int i=0; i<cloud->points.size(); i++){
		 cloud->points[i].x = strArray[i][0];
		 cloud->points[i].y = strArray[i][1];
		 cloud->points[i].z = strArray[i][2];
		 //cout << "x: " << cloud->points[i].x << " y: " << cloud->points[i].y << "z: " << cloud->points[i].z << endl;
	}
}

void viewcloud (pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,string id){
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_color (cloud, r, g, b);
	pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> point_color (cloud);
	viewer.setBackgroundColor (0.0, 0.0, 0.0);
	viewer.addPointCloud(cloud,point_color,id);
	viewer.addCoordinateSystem();
}

void setparams (int & search_mode, double & scale1, double & scale2, double & neighbour1,\
		        double & neighbour2, double & threshold, double & segradius){
    cout << "Enter the mode, mode 1: scale, mode2: neighbour, ";
    cin >> search_mode;
    if (search_mode == 1){
        cout << "Enter scale1, scale2, threshold, segradius, ";
        cin >> scale1 >> scale2 >> threshold >> segradius;
    }
    else{
        cout << "Enter neighbour1, neighbour2, threshold, segradius, ";
        cin >> neighbour1 >> neighbour2 >> threshold >> segradius;
    }
}

void computenormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::search::Search<PointXYZ>::Ptr tree,\
		           int search_mode, double scale1, double scale2, double neighbour1, double neighbour2,\
				   pcl::PointCloud<PointNormal>::Ptr normals_small_scale,\
				   pcl::PointCloud<PointNormal>::Ptr normals_large_scale){
    if (cloud->isOrganized ()){
      tree.reset (new pcl::search::OrganizedNeighbor<PointXYZ> ());
    }
    else{
      tree.reset (new pcl::search::KdTree<PointXYZ> (false));
    }

    tree->setInputCloud (cloud);
    pcl::NormalEstimation<PointXYZ, PointNormal> ne;
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    if (search_mode ==1){
        cout << "Calculating normals for scale..." << scale1 << endl;
        ne.setRadiusSearch (scale1);
        ne.compute (*normals_small_scale);
        cout << "Calculating normals for scale..." << scale2 << endl;
        ne.setRadiusSearch (scale2);
        ne.compute (*normals_large_scale);
    }
    else{
        cout << "Calculating normals for scale..." << neighbour1 << endl;
        ne.setKSearch(neighbour1);
        ne.compute (*normals_small_scale);
        cout << "Calculating normals for scale..." << neighbour2 << endl;
        ne.setKSearch(neighbour2);
        ne.compute (*normals_large_scale);
    }
}

void computedon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,PointCloud<PointNormal>::Ptr doncloud,\
		        pcl::PointCloud<PointNormal>::Ptr normals_small_scale,\
				pcl::PointCloud<PointNormal>::Ptr normals_large_scale){
    // Create output cloud for DoN results
    copyPointCloud<PointXYZ, PointNormal>(*cloud, *doncloud);
    cout << "Calculating DoN... " << endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
    don.setInputCloud (cloud);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);
    if (!don.initCompute ())
    {
      std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
      exit (EXIT_FAILURE);
    }
    // Compute DoN
    don.computeFeature (*doncloud);
    // Save DoN features
    //pcl::PCDWriter writer;
    //writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false);
}

void segcloud(PointCloud<PointNormal>::Ptr doncloud, double segradius,\
		      pcl::search::KdTree<PointNormal>::Ptr segtree,\
			  pcl::EuclideanClusterExtraction<PointNormal> ec,\
			  std::vector<pcl::PointIndices> & cluster_indices){
    cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;
    segtree->setInputCloud (doncloud);
    ec.setClusterTolerance (segradius);
    ec.setMinClusterSize (20);
    ec.setMaxClusterSize (500000);
    ec.setSearchMethod (segtree);
    ec.setInputCloud (doncloud);
    ec.extract (cluster_indices);
}

void viewseg(PointCloud<PointNormal>::Ptr doncloud, std::vector<pcl::PointIndices> & cluster_indices,\
		     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,\
		     std::string cloud_id, int j, pcl::visualization::PCLVisualizer & viewer){
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); \
                                                        it != cluster_indices.end (); ++it, j++){
    	pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        	cloud_cluster_don->points.push_back (doncloud->points[*pit]);
        }
        cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
        cloud_cluster_don->height = 1;
        cloud_cluster_don->is_dense = true;
        //Save cluster
        cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
        //ss << "don_cluster_" << j;
        cloud2->clear();
        pcl::copyPointCloud<PointNormal,PointXYZ>(*cloud_cluster_don,*cloud2);
        cloud_id = "cloud " + std::to_string(j);
        viewcloud (viewer, cloud2, cloud_id);
        cout << cloud_id << endl;
      }
}

void cloudfilter(pcl::PointCloud<PointNormal>::Ptr & doncloud_filtered, double threshold,\
		         PointCloud<PointNormal>::Ptr & doncloud){
    cout << "Filtering out DoN mag <= " << threshold << "..." << endl;
    pcl::ConditionOr<PointNormal>::Ptr range_cond (new pcl::ConditionOr<PointNormal> ());
    range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                               new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold)));
    // Build the filter
    pcl::ConditionalRemoval<PointNormal> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (doncloud);
    // Apply filter
    condrem.filter (*doncloud_filtered);
    doncloud = doncloud_filtered;
}


int main(){

	int search_mode;
    double scale1, scale2;
    double neighbour1,neighbour2;
    double threshold,segradius;
    string file_dir;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);
    pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);
    pcl::search::Search<PointXYZ>::Ptr tree;
    PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
    pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointNormal> ec;
    pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

    int j = 0;
    std::string cloud_id;

    cout << "Enter the file directory" << endl;
    cin >> file_dir;
    //  /home/wuhang/sydney_pointcloud/sydney-urban-objects-dataset/scans/scan.270.csv
    get2pcl (file_dir, cloud);
    setparams (search_mode,scale1,scale2,neighbour1,neighbour2,threshold,segradius);

    computenormal(cloud, tree, search_mode, scale1, scale2, neighbour1, neighbour2, \
    			  normals_small_scale, normals_large_scale);
    computedon(cloud,doncloud,normals_small_scale,normals_large_scale);
    cloudfilter(doncloud_filtered, threshold, doncloud);
    segcloud(doncloud, segradius,segtree,ec,cluster_indices);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewseg(doncloud, cluster_indices, cloud2, cloud_id, j, viewer);

	while (!viewer.wasStopped ()){
	    viewer.spinOnce ();
	}

    cout << "Done!" << endl;
    return 0;
}

