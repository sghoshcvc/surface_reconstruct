#include <CGAL/Simple_cartesian.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <utility>  // std::tuple
#include <vector>

// CGAL using declarations
using Kernel = CGAL::Simple_cartesian<float>;

using Point = Kernel::Point_3; 
using Vector = Kernel::Vector_3;
using Color = std::array<unsigned char, 1>; 

using PNC = std::tuple<Point, Vector, Color>;

// PCL using declarations
using Cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>;

std::vector<PNC> pcl2cgal(const Cloud & cloud)
{
  std::vector<PNC> points;
  points.reserve(cloud.points.size());

  for (const auto & pt : cloud.points) {
    Point p (pt.x, pt.y, pt.z );
    Vector n (pt.normal_x, pt.normal_y, pt.normal_z);
    Color c { {0} };

    points.push_back(std::make_tuple(p, n, c));
  }

  return points;
}

Cloud cgal2pcl(const std::vector<PNC> & points)
{
  Cloud cloud;

  for (const auto & pnc : points) {
    const Point & p = std::get<0>(pnc);
    const Vector & v = std::get<1>(pnc);
    const Color & c = std::get<2>(pnc);

    pcl::PointXYZRGBNormal pt; 
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    pt.normal_x = v.x();
    pt.normal_y = v.y();
    pt.normal_z = v.z();
    pt.r = c[0];
    pt.g = c[1];
    pt.b = c[2];
    cloud.points.push_back(pt);
  }

  return cloud;
}

int main (int argc, char** argv) 
{
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
pcl::PCLPointCloud2 cloud_blob;
pcl::io::loadPCDFile (argv[1], cloud_blob); 
pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  // Normal estimation* 
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n; 
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); 
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud (cloud); 
  n.setInputCloud (cloud); 
  n.setSearchMethod (tree); 
  n.setKSearch (20); 
  n.compute (*normals); 
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals); 
  //std::vector<PNC> points;
  //points = pcl2cgal(*cloud_with_normals)
  
  std::vector<PNC> points;
  points.reserve(cloud_with_normals->points.size());

  for (const auto & pt : cloud_with_normals->points) {
    Point p (pt.x, pt.y, pt.z );
    Vector n (pt.normal_x, pt.normal_y, pt.normal_z);
    Color c { {0} };

    points.push_back(std::make_tuple(p, n, c));
  }
 }
