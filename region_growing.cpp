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

#include<string>

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
  ss = "/home/pragna/segmented_pcd//"+ss ;
  writer.write<pcl::PointXYZRGB> (ss, *cloud_segmented, false);


  if (pcl::console::find_switch (argc, av, "-dump"))
  {
    pcl::console::print_highlight ("Writing clusters to clusters.dat\n");
    std::ofstream clusters_file;
    std::ofstream centroid_file;
    
    std::string fname=av[1];
    std::string cl = ".clusters.dat";
    std::string cen = ".centroid.dat";
    //clusters_file.open (fname+cl);
    centroid_file.open (fname+cen);
    for (std::size_t i = 0; i < clusters.size (); ++i)
    {
      if (clusters[i].indices.size() >=100)
      {
        centroid_file << av[1];
        //clusters_file << i << "#" << clusters[i].indices.size () << ": ";
        //centroid_file << i << "#" << clusters[i].indices.size () << ": ";
        //std::vector<int>::const_iterator pit = clusters[i].indices.begin ();
        //clusters_file << *pit;
        Eigen::Vector4f centroid,min_pt,max_pt;
      
        pcl::compute3DCentroid(*cloud_filtered2,clusters[i].indices,centroid);
        pcl::getMinMax3D(*cloud_filtered2,clusters[i].indices,min_pt,max_pt);
      
        centroid_file << "," << centroid[0] << "," << centroid[1] << "," << centroid[2];
        centroid_file << "," << max_pt[0]-min_pt[0] << "," << max_pt[1]-min_pt[1] << "," << max_pt[2]-min_pt[2];
        centroid_file << "," << std::max(max_pt[0]-min_pt[0],std::max(max_pt[1]-min_pt[1], max_pt[2]-min_pt[2]));
        centroid_file << std::endl;
       }
       
      //for (; pit != clusters[i].indices.end (); ++pit)
        //clusters_file << " " << *pit;
      //clusters_file << std::endl;
    }
    clusters_file.close ();
    centroid_file.close();
  }

  return (0);
}
