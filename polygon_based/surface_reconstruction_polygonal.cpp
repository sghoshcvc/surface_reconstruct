#include <CGAL/Simple_cartesian.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<fstream>
#include <utility>  // std::tuple
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/IO/Writer_OFF.h>
#include <CGAL/property_map.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Polygonal_surface_reconstruction.h>
#ifdef CGAL_USE_SCIP  // defined (or not) by CMake scripts, do not define by hand
#include <CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double>                        MIP_Solver;
#elif defined(CGAL_USE_GLPK)  // defined (or not) by CMake scripts, do not define by hand
#include <CGAL/GLPK_mixed_integer_program_traits.h>
typedef CGAL::GLPK_mixed_integer_program_traits<double>                        MIP_Solver;
#endif
#if defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)
#include <CGAL/Timer.h>
#include <fstream>
typedef CGAL::Exact_predicates_inexact_constructions_kernel                        Kernel;
typedef Kernel::Point_3                                                                                                Point;
typedef Kernel::Vector_3                                                                                        Vector;
// Point with normal, and plane index
//typedef std::tuple<Point, Vector, int> PNI;
typedef boost::tuple<Point, Vector, int>                                                        PNI;
typedef std::vector<PNI>                                                                                        Point_vector;
typedef CGAL::Nth_of_tuple_property_map<0, PNI>                                                Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNI>                                                Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNI>                                                Plane_index_map;
typedef CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, Point_vector, Point_map, Normal_map>     Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits>             Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits>                                                Plane;
typedef CGAL::Shape_detection::Point_to_shape_index_map<Traits>     Point_to_shape_index_map;
typedef        CGAL::Polygonal_surface_reconstruction<Kernel>                                Polygonal_surface_reconstruction;
typedef CGAL::Surface_mesh<Point>                                                                        Surface_mesh;

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
  
  std::vector<PNI> points;
  points.reserve(cloud_with_normals->points.size());
  //std::cout << "Extracting points..." << std::str(cloud_with_normals->points.size());
  pcl::console::print_highlight ("Found %lu points\n", cloud_with_normals->points.size ());
  std::size_t i = 0;
  std::ofstream myfile;
  myfile.open ("tile.pwn");
  for (const auto & pt : cloud_with_normals->points) {
    Point p (pt.x, pt.y, pt.z );
    Vector n (pt.normal_x, pt.normal_y, pt.normal_z);
    int c =0;
    points[i].get<0>() = p;
    points[i].get<1>() = n;
    points[i].get<2>() = c;
    i++;
    myfile << pt.x << " "<< pt.y << " " << pt.z << " " << pt.normal_x << " "<< pt.normal_y << " " << pt.normal_z << std::endl;

    //points.push_back(std::make_tuple(p, n, c));
  }
  myfile.close();
          CGAL::Timer t;
        t.start();
  std::cout << "starting RANSAC...";
          // Shape detection
        Efficient_ransac ransac;
        ransac.set_input(points);
        ransac.add_shape_factory<Plane>();
        std::cout << "Extracting planes...";
        t.reset();
        ransac.detect();
        Efficient_ransac::Plane_range planes = ransac.planes();
        std::size_t num_planes = planes.size();
        std::cout << " Done. " << num_planes << " planes extracted. Time: " << t.time() << " sec." << std::endl;
        // Stores the plane index of each point as the third element of the tuple.
        Point_to_shape_index_map shape_index_map(points, planes);
        for (std::size_t i = 0; i < points.size(); ++i) {
                // Uses the get function from the property map that accesses the 3rd element of the tuple.
                int plane_index = get(shape_index_map, i);
                points[i].get<2>() = plane_index;
        }
        std::cout << "Generating candidate faces...";
        t.reset();
        Polygonal_surface_reconstruction algo(
                points,
                Point_map(),
                Normal_map(),
                Plane_index_map()
        );
        std::cout << " Done. Time: " << t.time() << " sec." << std::endl;
        Surface_mesh model;
        std::cout << "Reconstructing...";
        t.reset();
        if (!algo.reconstruct<MIP_Solver>(model)) {
                std::cerr << " Failed: " << algo.error_message() << std::endl;
                return EXIT_FAILURE;
        }
        const std::string& output_file("output.off");
        std::ofstream output_stream(output_file.c_str());
        if (output_stream && CGAL::write_off(output_stream, model)) {
                // flush the buffer
                output_stream << std::flush;
                std::cout << " Done. Saved to " << output_file << ". Time: " << t.time() << " sec." << std::endl;
        }
        else {
                std::cerr << " Failed saving file." << std::endl;
                return EXIT_FAILURE;
        }
        // Also stores the candidate faces as a surface mesh to a file
        Surface_mesh candidate_faces;
        algo.output_candidate_faces(candidate_faces);
        const std::string& candidate_faces_file("output_faces.off");
        std::ofstream candidate_stream(candidate_faces_file.c_str());
        if (candidate_stream && CGAL::write_off(candidate_stream, candidate_faces)) {
                // flush the buffer
                output_stream << std::flush;
                std::cout << "Candidate faces saved to " << candidate_faces_file << "." << std::endl;
        }
        return EXIT_SUCCESS;
}
#else
int main(int, char**)
{
    std::cerr << "This test requires either GLPK or SCIP.\n";
    return EXIT_SUCCESS;
}

#endif 
