#pragma once
//Depends: PCL 1.2, Boost 1.66, Eigen3, Search
#include <vector>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/octree/octree_search.h>
#include <voronoi_route_planner/Search.hpp>

class Voronoi_graph : public Searchable<std::vector<double>, double>
{
    private:
        double conflict_zone;//vehicle conflict radius
        std::vector<double> end;//end being searched for
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr world;//needed for voronoi construction and nav termination
        std::map<std::pair<double,double>,std::vector<std::pair<double,double>>> roadMap;//storing voronoi edges for look up
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr voronoiVertex;//octree search structure needed for roadmap onboarding
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertexList;//vertex coordinates needed for roadmap onboarding

        void constructRoadmap();

        bool isEdgeFree(pcl::PointXYZ p, pcl::PointXYZ q);

    public:

        Voronoi_graph(double conflict_radius, std::vector<double> end, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr map);

        ~Voronoi_graph();

        void setEnd(std::vector<double> endNode);

        double getNul() override;

        double getHzn() override;

        double callHrst(std::vector<double> node, std::vector<double> end) override;

        std::vector<std::pair<std::vector<double>,double>> edgesFrom(std::vector<double> node) override;

        pcl::PointCloud<pcl::PointXYZ>::Ptr getRoadMap();

        pcl::PointCloud<pcl::PointXYZ>::Ptr getWorld();
};