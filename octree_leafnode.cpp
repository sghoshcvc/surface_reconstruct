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
#include<pcl/octree/octree.h>
#include<pcl/octree/octree_iterator.h>

bool sortinrev(const std::pair<int,int> &a,  
               const std::pair<int,int> &b) 
{ 
       return (a.first > b.first); 
} 

int
main (int argc, char** av)
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
  //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  //ne.setInputCloud (cloud_filtered2);
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>());
  //ne.setSearchMethod (tree_n);
  //ne.setRadiusSearch (0.03);
  //ne.compute (*cloud_normals);
  //pcl::console::print_highlight ("Normals are computed and size is %lu\n", cloud_normals->points.size ());

  // create octree
    
  pcl::octree::OctreePointCloud<pcl::PointXYZ> octreeA( 0.1f );
  octreeA.setInputCloud( cloud_filtered2 );
  octreeA.addPointsFromInputCloud(); 
    // instantiate iterator for octreeA
  //pcl::octree::OctreePointCloud<pcl::PointXYZ>::LeafNodeIterator it (octreeA);
  //pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ>::LeafNodeBreadthFirstIterator it(octreeA);

  

  if (pcl::console::find_switch (argc, av, "-dump"))
  {
    pcl::console::print_highlight ("Writing clusters to clusters.dat\n");
    std::ofstream clusters_file;
    std::ofstream centroid_file;
    
    std::string fname=av[1];
    //std::string cl = ".clusters.dat";
    std::string cen = ".centroid_octree.dat";
    //clusters_file.open (fname+cl);
    centroid_file.open (fname+cen);
    int i =0;
    
    std::vector<std::vector<int> > clusters;
    std::vector<std::pair<int, int> > vp;
    for (auto it = octreeA.leaf_begin(); it != octreeA.leaf_end(); ++it)
    {
      std::vector<int> indexVector;
      it.getLeafContainer().getPointIndices(indexVector);
      clusters.push_back(indexVector);
      vp.push_back(std::make_pair(indexVector.size(),i));
      i++;
    }
    std::sort(vp.begin(), vp.end(),sortinrev); 
    for (int i = 0; i < 10; i++) 
    {

      //Eigen::Vector3f voxel_min, voxel_max;
      //octreeA.getVoxelBounds(it, voxel_min, voxel_max);
      //it.getData (indexVector);
      //clusters_file << i << "#" << clusters[i].indices.size () << ": ";
      //centroid_file << i << "#" << indexVector.size () << ": ";
      //std::vector<int>::const_iterator pit = clusters[i].indices.begin ();
      //clusters_file << *pit;
      centroid_file << av[1];
      Eigen::Vector4f centroid, min_pt,max_pt;
      
      pcl::compute3DCentroid(*cloud_filtered2,clusters[vp[i].second],centroid);    
      pcl::getMinMax3D(*cloud_filtered2,clusters[vp[i].second],min_pt,max_pt);
      centroid_file << "," << centroid[0] << "," << centroid[1] << "," << centroid[2];
      centroid_file << "," << max_pt[0]-min_pt[0] << "," << max_pt[1]-min_pt[1] << "," << max_pt[2]-min_pt[2];
      centroid_file << "," << std::max(max_pt[0]-min_pt[0],std::max(max_pt[1]-min_pt[1], max_pt[2]-min_pt[2]));
      centroid_file << std::endl;
      
       
      //for (; pit != clusters[i].indices.end (); ++pit)
       // clusters_file << " " << *pit;
      //clusters_file << std::endl;
    }
    //clusters_file.close ();
    centroid_file.close();
  }

  return (0);
}
