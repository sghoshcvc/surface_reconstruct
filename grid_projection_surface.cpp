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
  pcl::console::print_highlight ("Before Normal estimation");
       


  // Concatenate the XYZ and normal fields* 
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals); 
  pcl::console::print_highlight ("After Normal Estimation");
       

  // Create search tree* 
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>); 
  tree2->setInputCloud (cloud_with_normals); 
  pcl::console::print_highlight ("Before surface Reconstruction");

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
  pcl::console::print_highlight ("before parsing");
  pcl::io::savePLYFile("tile_surface.ply", triangles);

  // linear scan through polygons
  for(int i=0;i<triangles.polygons.size();i++){
    pcl::Vertices currentPoly = triangles.polygons[i];
    // for each Vertex find robot position???
    //for(int ii=0;ii<currentPoly.vertices.size();ii++){
      //pcl::PointNormal currentPt = pcl::PointNormal();
      //int x = currentPoly.vertices[ii].x;
      //int y = currentPoly.vertices[ii].y;
      //int z = currentPoly.vertices[ii].z;
      //pcl::console::print_highlight ("vNumber of ertex"+ std::to_string(currentPoly.vertices.size()));
      std::cout << " Number of Vertices " << currentPoly.vertices.size() << std::endl;

      //}                          //outputCloud->points.push_back(currentPt);//push in points without normals
                     
  }



  // Finish 
  return (0);
}
