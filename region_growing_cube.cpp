/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id:$
 *
 */

#include <iostream>
#include<string>

#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
 #include <pcl/common/eigen.h>
#include <pcl/common/impl/transforms.hpp>

#include<string>
typedef pcl::PointXYZ PointT;

using std::string;
string getFileName(const string& s) {

   char sep = '/';

#ifdef _WIN32
   sep = '\\';
#endif

   size_t i = s.rfind(sep, s.length());
   if (i != string::npos) {
      return(s.substr(i+1, s.length() - i));
   }

   return("");
}


int main (int argc, char** av)
{
  if (argc < 2)
  {
    pcl::console::print_info ("Syntax is: %s <source-pcd-file> [-dump]\n\n", av[0]);
    pcl::console::print_info ("If -dump is provided write the extracted clusters to cluster.dat\n\n");
    return (1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_nans (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PCDWriter writer;
  if (pcl::io::loadPCDFile(av[1], *cloud_ptr)==-1)
  {
    return -1;
  }

  pcl::console::print_highlight ("Loaded cloud %s of size %lu\n", av[1], cloud_ptr->points.size ());

  // Remove the nans
  cloud_ptr->is_dense = false;
  cloud_no_nans->is_dense = false;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud (*cloud_ptr, *cloud_no_nans, indices);
  pcl::console::print_highlight ("Removed nans from %lu to %lu\n", cloud_ptr->points.size (), cloud_no_nans->points.size ());
//remove statistical outliers 
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_no_nans);
  sor.setMeanK (20);
  sor.setStddevMulThresh (2.0);
  sor.filter (*cloud_filtered1);
  
  pcl::VoxelGrid<pcl::PointXYZ> uor;
  uor.setInputCloud (cloud_filtered1);
  uor.setLeafSize (0.02f, 0.02f, 0.02f); 
  uor.filter (*cloud_filtered2);


  // Estimate the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_filtered2);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod (tree_n);
  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);
  pcl::console::print_highlight ("Normals are computed and size is %lu\n", cloud_normals->points.size ());

  // Region growing
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
  rg.setSmoothModeFlag (false); // Depends on the cloud being processed
  rg.setInputCloud (cloud_filtered2);
  rg.setInputNormals (cloud_normals);

  std::vector <pcl::PointIndices> clusters;
  pcl::StopWatch watch;
  rg.extract (clusters);
  pcl::console::print_highlight ("Extraction time: %f\n", watch.getTimeSeconds());
  cloud_segmented = rg.getColoredCloud ();
  
  //pcl::visualization::CloudViewer viewer ("Cluster viewer");
  //viewer.showCloud(cloud_segmented);
  //while (!viewer.wasStopped ())
  //{
  //}

  // Writing the resulting cloud into a pcd file
  pcl::console::print_highlight ("Number of segments done is %lu\n", clusters.size ());
  std::string ss;
  ss = getFileName(av[1]);
  //ss = "/home/pragna/segmented_pcd//"+ss ;
  //writer.write<pcl::PointXYZRGB> (ss, *cloud_segmented, false);
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 


  if (pcl::console::find_switch (argc, av, "-dump"))
  {
    pcl::console::print_highlight ("Writing clusters to clusters.dat\n");
    std::ofstream clusters_file;
    std::ofstream centroid_file;
    
    std::string fname=av[1];
    std::string cl = ".clusters.dat";
    std::string cen = ".centroid.dat";
    //clusters_file.open (fname+cl);
    //centroid_file.open (fname+cen);
    int j =0;
    pcl::visualization::PCLVisualizer viewer;
    //setRepresentationToWireframeForAllActors()
    
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
    {
      if (it->indices.size() >=10)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
         pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
         pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>());
        for (const int &index : it->indices)
           cloud_cluster->push_back ((*cloud_filtered2)[index]); 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        std::stringstream ss,tt,cc;
        ss << "cluster_cloud" << j;
        //std::stringstream tt;
        tt << "cluster_bbox" << j;
        cc << "cylinder" << j ; 
        
   //pcl::PointCloud<PointT>::Ptr point_cloud_ptr = ...;

    // compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud_cluster, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<PointT> cPoints;
    pcl::transformPointCloud(*cloud_cluster, cPoints, p2w);

    PointT min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // final transform
    const Eigen::Quaternionf qfinal(eigDx);
    const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

    

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_cluster);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod (tree_n);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals1);
  pcl::console::print_highlight ("Normals are computed and size is %lu\n", cloud_normals1->points.size ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_cluster);
    seg.setInputNormals (cloud_normals1);

   seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_cluster);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
    
    // draw the cloud and the box
    viewer.addPointCloud(cloud_cluster,ss.str());
    viewer.addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z,tt.str());
    viewer.addCylinder(*coefficients_cylinder, cc.str());
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.7, 0.7, 0, tt.str());
    viewer.setRepresentationToWireframeForAllActors(); 
    viewer.spin();
    j++;




       }    

     }
       
      //for (; pit != clusters[i].indices.end (); ++pit)
        //clusters_file << " " << *pit;
      //clusters_file << std::endl;
   

  return (0);
}
}
