#include <vector>
#include <algorithm>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/octree/octree_search.h>
#include <boost/polygon/voronoi.hpp>
#include <cca_utilities/Voronoi_graph.hpp>

Voronoi_graph::Voronoi_graph(double conflict_radius, std::vector<double> end, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr map): 
conflict_zone(conflict_radius), end(end), world(std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(*map))//world init uses copy constructor for independence
{
    this->voronoiVertex = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(map->getResolution());
    this->vertexList = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->constructRoadmap();
}

Voronoi_graph::~Voronoi_graph(){}

void Voronoi_graph::constructRoadmap()
{
    //init voronoi
    std::vector<boost::polygon::point_data <int>> diag_pnts;
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> voxelList;
    this->world->getOccupiedVoxelCenters(voxelList);
    for(auto vxl: voxelList){diag_pnts.push_back(boost::polygon::point_data<int>((int)(vxl.x/this->world->getResolution()),(int)(vxl.y/this->world->getResolution())));}//discretize and add
    
    //run voronoi
    boost::polygon::voronoi_diagram<double> vd;
    boost::polygon::construct_voronoi(diag_pnts.begin(),diag_pnts.end(),&vd);
    
    //extract non conflicting primary edges
    for(boost::polygon::voronoi_diagram<double>::const_edge_iterator e = vd.edges().begin(); e != vd.edges().end(); e++){//iterate over edges
        if(e->is_primary() && e->vertex0() != nullptr && e->vertex1() != nullptr){//edge validity check
            //access data
            pcl::PointXYZ p(this->world->getResolution() * e->vertex0()->x(), this->world->getResolution() * e->vertex0()->y(),end[2]);
            pcl::PointXYZ q(this->world->getResolution()* e->vertex1()->x(), this->world->getResolution()* e->vertex1()->y(),end[2]);
            if(this->isEdgeFree(p,q)){//build road map
                std::pair<double,double> v0(p.x,p.y);
                std::pair<double,double> v1(q.x,q.y);
                //add both directions to road map
                this->roadMap[v0].push_back(v1);
                this->roadMap[v1].push_back(v0);
            }
        }
    }

    //build vertex point cloud
    this->vertexList = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto mapV = this->roadMap.begin(); mapV != this->roadMap.end(); mapV++){
        pcl::PointXYZ v(mapV->first.first,mapV->first.second,end[2]);//iterate over keys
        this->vertexList->push_back(v);//add point
    }
    
    //build vertex search object
    this->voronoiVertex->setInputCloud(this->vertexList);
    this->voronoiVertex->addPointsFromInputCloud();
}

bool Voronoi_graph::isEdgeFree(pcl::PointXYZ p, pcl::PointXYZ q)
{
    pcl::Indices ind;
    std::vector<float> fs({});
    double l = std::sqrt((q.x-p.x)*(q.x-p.x)+(q.y-p.y)*(q.y-p.y)+(q.z-p.z)*(q.z-p.z));
    double t = 0.0;
    while (t < 1.0 + (this->conflict_zone/l)){//walk edge
        t = std::min(t,1.0);
        auto n = this->world->radiusSearch(pcl::PointXYZ(t*(q.x-p.x)+p.x,t*(q.y-p.y)+p.y,t*(q.z-p.z)+p.z),this->conflict_zone,ind,fs);
        if(n > 0){return false;}
        t+=(this->conflict_zone/l);
    }
    return true;
}

void Voronoi_graph::setEnd(std::vector<double> endNode){this->end = endNode;}

double Voronoi_graph::getNul() {return 0.0;}

double Voronoi_graph::getHzn() {return (double) std::numeric_limits<double>::infinity();}

double Voronoi_graph::callHrst(std::vector<double> node, std::vector<double> end) {return std::sqrt(std::max(((end[0] - node[0])*(end[0] - node[0]) + (end[1] - node[1])*(end[1] - node[1])), 0.0));}

std::vector<std::pair<std::vector<double>,double>> Voronoi_graph::edgesFrom(std::vector<double> node) 
{
    //prepare data structures
    std::vector<std::pair<std::vector<double>,double>> edges;//edges to return
    
    //empty map fault condition
    if (this->world->getLeafCount() == 0){
        edges.emplace_back(this->end,this->callHrst(node, this->end));
        return edges;
    }

    //check if end is in sight
    pcl::PointXYZ currentPoint(node[0], node[1], node[2]);
    pcl::PointXYZ endPoint(this->end[0], this->end[1], currentPoint.z);
    if(this->isEdgeFree(currentPoint,endPoint)){//include end if visible
        edges.emplace_back(this->end,this->callHrst(node, this->end));
    }

    //navigate voronoi
    std::pair<float,float> pnt(node[0],node[1]);
    auto p = node;
    if(this->roadMap.count(pnt) > 0){//check if currently on roadmap
        for(auto v: this->roadMap[pnt]){//iterate over accessible verteces
            p[0] = v.first;
            p[1] = v.second;
            edges.emplace_back(p,this->callHrst(node,p));
        }
    }else{//find visible roadmap vertex
        for(auto v: this->roadMap){
            pcl::PointXYZ v_pt(v.first.first,v.first.second,currentPoint.z);
            if(this->isEdgeFree(currentPoint,v_pt)){
                p[0] = v.first.first;
                p[1] = v.first.second;
                edges.emplace_back(p,this->callHrst(node,p));
            }
        }
    }

    return edges;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Voronoi_graph::getRoadMap(){return this->vertexList;}

pcl::PointCloud<pcl::PointXYZ>::Ptr Voronoi_graph::getWorld()
{
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> occupiedVoxelList;
    this->world->getOccupiedVoxelCenters(occupiedVoxelList);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedMap(new pcl::PointCloud<pcl::PointXYZ>);
    projectedMap->assign(occupiedVoxelList.begin(),occupiedVoxelList.end());
    return projectedMap;
}