#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <stdio.h>

int main (int argc, char** argv) 
{ 
        // Load input file into a PointCloud<T> with an appropriate type 
//pcl::console::print_highlight ("here 1");

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
pcl::PCLPointCloud2 cloud_blob;
pcl::io::loadPCDFile (argv[1], cloud_blob); 
        
pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  // Normal estimation* 
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n; 
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); 
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); 
  tree->setInputCloud (cloud); 
  n.setInputCloud (cloud); 
  n.setSearchMethod (tree); 
  n.setKSearch (20); 
  n.compute (*normals); 
//  pcl::console::print_highlight ("here 2");
       


  // Concatenate the XYZ and normal fields* 
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals); 
  //pcl::console::print_highlight ("here 3");
       

  // Create search tree* 
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>); 
  tree2->setInputCloud (cloud_with_normals); 
  //pcl::console::print_highlight ("here 4");

        // Initialize objects 
        pcl::GridProjection<pcl::PointNormal> gbpolygon; 
        pcl::PolygonMesh triangles; 

        // Set parameters 
        gbpolygon.setResolution(0.02); 
        gbpolygon.setPaddingSize(3); 
        gbpolygon.setNearestNeighborNum(5000); 
        gbpolygon.setMaxBinarySearchLevel(100); 
        
        // Get result 
        gbpolygon.setInputCloud(cloud_with_normals); 
        gbpolygon.setSearchMethod(tree2); 
        gbpolygon.reconstruct(triangles); 
        
  // For Visualization 
  pcl::io::savePLYFile("tile_surface.ply", triangles); 

  // Finish 
  return (0);
}
