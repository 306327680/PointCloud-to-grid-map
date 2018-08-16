#include <iostream>
#include <fstream>
#include <stdint.h>
#include <math.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/features/intensity_gradient.h>
#include <dynamic_reconfigure/server.h>
#include <cloud_to_map/cloud_to_map_nodeConfig.h>

double threshold_z1 = 0;  // should add a function to de terimnine the ground automatically
int diffThresh = 4;       // the value above certain height - the value under certain height
int greenThresh;          // the thereshold of "may we can pass"
int orangeThresh = 20;         // the thereshold of "high obstacle"
int redThresh = 40;            // the thereshold of "very high obstacle"
int wallThresh =10;           // the thereshold of "can't pass obstacle"

/* Define the two point cloud types used in this code */
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

/* Global */
PointCloud::ConstPtr currentPC;
bool newPointCloud = false;
bool reconfig = false;


// -----------------------------------------------
// -----Define a structure to hold parameters-----
// -----------------------------------------------
struct Param {
  std::string frame;
  double searchRadius;
  double deviation;
  int buffer;
  double loopRate;
  double cellResolution;
  
};

// -----------------------------------
// -----Define default parameters-----
// -----------------------------------
Param param;
boost::mutex mutex;
void loadDefaults(ros::NodeHandle& nh) {
  nh.param<std::string>("frame", param.frame, "map");
  nh.param("search_radius", param.searchRadius, 1.0);
  nh.param("deviation", param.deviation, 0.1);
  nh.param("buffer", param.buffer, 5);
  nh.param("loop_rate", param.loopRate, 10.0);
  nh.param("cell_resolution", param.cellResolution, 0.3);
 // nh.param("z1_thereshold", param.threshold_z1, 1);
}
// -----------------------------------------------
// -----  To find the ground abtomatcally    -----
// -----------------------------------------------
void groundZfinder(){
  std::vector<std::vector<double> > groundDifReg;
  
   


}
// ------------------------------------------------------
// -----Update current PointCloud if msg is received-----
// ------------------------------------------------------
void callback(const PointCloud::ConstPtr& msg) {
  boost::unique_lock<boost::mutex>(mutex);
  currentPC = msg;
  newPointCloud = true;
}

// ------------------------------------------
// -----Callback for Dynamic Reconfigure-----
// ------------------------------------------
void callbackReconfig(cloud_to_map::cloud_to_map_nodeConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %s %f %f %f %f", config.frame.c_str(), config.deviation,
      config.loop_rate, config.cell_resolution, config.search_radius);
  boost::unique_lock<boost::mutex>(mutex);
  param.frame = config.frame.c_str();
  param.searchRadius = config.search_radius;
  param.deviation = config.deviation;
  param.buffer = config.buffer;
  param.loopRate = config.loop_rate;
  param.cellResolution = config.cell_resolution;
  reconfig = true;
}

// ----------------------------------------------------------------
// -----Calculate surface normals with a search radius of 0.05-----
// ----------------------------------------------------------------
void calcSurfaceNormals(PointCloud::ConstPtr& cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(param.searchRadius);
  ne.compute(*normals);
}

// ---------------------------------------
// -----Initialize Occupancy Grid Msg-----
// ---------------------------------------
void initGrid(nav_msgs::OccupancyGridPtr grid) {
  grid->header.seq = 1;
  grid->header.frame_id = param.frame;
  grid->info.origin.position.z = 0;
  grid->info.origin.orientation.w = 1;
  grid->info.origin.orientation.x = 0;
  grid->info.origin.orientation.y = 0;
  grid->info.origin.orientation.z = 0;
}

// -----------------------------------
// -----Update Occupancy Grid Msg-----
// -----------------------------------
void updateGrid(nav_msgs::OccupancyGridPtr grid, double cellRes, int xCells, int yCells,
    double originX, double originY, std::vector<signed char> *ocGrid) {
  grid->header.seq++;
  grid->header.stamp.sec = ros::Time::now().sec;
  grid->header.stamp.nsec = ros::Time::now().nsec;
  grid->info.map_load_time = ros::Time::now();
  grid->info.resolution = cellRes;
  grid->info.width = xCells;
  grid->info.height = yCells;
  grid->info.origin.position.x = originX;
  grid->info.origin.position.y = originY;
  grid->data = *ocGrid;
}

// ------------------------------------------
// -----Calculate size of Occupancy Grid-----
// ------------------------------------------
void calcSize(double *xMax, double *yMax, double *xMin, double *yMin) {
  for (size_t i = 0; i < currentPC->size(); i++) {
    double x = currentPC->points[i].x;
    double y = currentPC->points[i].y;
    if (*xMax < x) {
      *xMax = x;
    }
    if (*xMin > x) {
      *xMin = x;
    }
    if (*yMax < y) {
      *yMax = y;
    }
    if (*yMin > y) {
      *yMin = y;
    }
  }
}

// ---------------------------------------
// -----Populate map with cost values-----
// ---------------------------------------
void populateMap(NormalCloud::Ptr cloud_normals, std::vector<int> &map,double xMax, double yMax, double xMin, double yMin,
    double cellResolution, int xCells, int yCells) {
    double deviation = param.deviation;  
    std::vector<std::vector<int> > map_n;
    std::cout<<"   1    "<<std::endl;
    int the_size_of_map;
    the_size_of_map = (xMax-xMin)*(yMax-yMin)/(cellResolution*cellResolution);

    int conditions=2;
    map_n.clear();
    map_n.resize(conditions);

    for(int j =0; j<conditions; j++)
    {
      map_n[j].resize(the_size_of_map); // whether it is not big enough to put the point cloud in？currentPC->size()/cellResolution
    }
std::cout<<"   2    "<<std::endl;
    //std::cout<< "test point 1"<<std::endl;
    //int x1 = currentPC->size()/(cellResolution*cellResolution);
  for (size_t i = 0; i < currentPC->size(); i++) 
  {
    double x = currentPC->points[i].x;
    double y = currentPC->points[i].y;
    double z = cloud_normals->points[i].normal_z;
    double z_axis = currentPC->points[i].z;

    double phi = acos(fabs(z));
    int xCell, yCell;
    //std::cout<< i <<" i,  the_size_of_map: "<<the_size_of_map<<"  the place to write in map_n "<<yCell * xCells + xCell<<std::endl;
    if (1) { //TODO implement cutoff height!!!
      xCell = (int) ((x - xMin) / cellResolution);
      yCell = (int) ((y - yMin) / cellResolution);

     // std::cout<< xCell  <<"  test point 3  "<<yCell <<std::endl;
      if ((yCell * xCells + xCell) > (xCells * yCells)) {
        std::cout << "x: " << x << ", y: " << y << ", xCell: " << xCell << ", yCell: " << yCell << "\n";
      }
      if (phi > deviation) 
      {
        if(z_axis<threshold_z1)
        {
          map_n[0][yCell * xCells + xCell] ++;
        }else
        {
          map_n[1][yCell * xCells + xCell] ++;
        }
          map[yCell * xCells + xCell]++;
      }

       if(map[yCell * xCells + xCell]==0) 
       {
        map[yCell * xCells + xCell] = 1;
       // map[yCell * xCells + xCell]--;
       }
    }
  }
  std::cout<<"   3    "<<std::endl;
   
      for(int m =0; m < currentPC->size(); m++)
    {
      if(map_n[0][m]>map_n[1][m]&&map_n[0][m] > diffThresh)
      {

      map[m] = -1;

      std::cout<<currentPC->size()<<"   4    "<<m<<std::endl;
      }
    }
     std::cout<<"   4    "<<std::endl;
    
  }



// ---------------------------------
// -----Generate Occupancy Grid-----
// -------------------else if (countGrid[i] < 40&&countGrid[i]>20) {

void genOccupancyGrid(std::vector<signed char> &ocGrid, std::vector<int> &countGrid, int size) {
  int buf = param.buffer;

  for (int i = 0; i < size; i++) {
    if (countGrid[i] < wallThresh&&countGrid[i]>0)
    {
      ocGrid[i] = 0;
    } else if (countGrid[i] > wallThresh&&countGrid[i]<orangeThresh)
    {
     // std::cout<<countGrid[i]<<"    ";
      ocGrid[i] = 100;
    } else if (countGrid[i] == 0) 
    {
      ocGrid[i] = -1; // TODO Should be -1
    } else if (countGrid[i] < redThresh&&countGrid[i]>orangeThresh)
    {
      ocGrid[i] = 200; // TODO Should be -1
    } else if (countGrid[i]>redThresh)
    {
      ocGrid[i] = 150; // TODO Should be -1
    } else if (countGrid[i]=-1)
    {
      ocGrid[i] = 101;//125
    }

  }
}

// --------------
// -----Main-----
// --------------
int main(int argc, char** argv) {
  /* Initialize ROS */
  ros::init(argc, argv, "cloud_to_map_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/voxel_filter", 1, callback);
  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);

  /* Initialize Dynamic Reconfigure */
  dynamic_reconfigure::Server<cloud_to_map::cloud_to_map_nodeConfig> server;
  dynamic_reconfigure::Server<cloud_to_map::cloud_to_map_nodeConfig>::CallbackType f;
  f = boost::bind(&callbackReconfig, _1, _2);
  server.setCallback(f);

  /* Initialize Grid */
  nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid);
  initGrid(grid);

  /* Finish initializing ROS */
  mutex.lock();
  ros::Rate loop_rate(param.loopRate);
  mutex.unlock();

  /* Wait for first point cloud */
  while(ros::ok() && newPointCloud == false){
    ros::spinOnce();
    loop_rate.sleep();
  }

  /* Begin processing point clouds */
  while (ros::ok()) {
    ros::spinOnce();
    boost::unique_lock<boost::mutex> lck(mutex);
    if (newPointCloud || reconfig) {
      /* Update configuration status */
      reconfig = false;
      newPointCloud = false;

      /* Record start time */
      uint32_t sec = ros::Time::now().sec;
      uint32_t nsec = ros::Time::now().nsec;

      /* Calculate Surface Normals */
      NormalCloud::Ptr cloud_normals(new NormalCloud);
      calcSurfaceNormals(currentPC, cloud_normals);

      /* Figure out size of matrix needed to store data. */
      double xMax = 0, yMax = 0, xMin = 0, yMin = 0;
      calcSize(&xMax, &yMax, &xMin, &yMin);
      std::cout << "xMax: " << xMax << ", yMax: " << yMax << ", xMin: " << xMin << ", yMin: "
          << yMin << "\n";

      /* Determine resolution of grid (m/cell) */
      double cellResolution = param.cellResolution;
      int xCells = ((int) ((xMax - xMin) / cellResolution)) + 1;
      int yCells = ((int) ((yMax - yMin) / cellResolution)) + 1;
      std::cout << "xCells: " << xCells << ", yCells: " << yCells << "\n";

      /* Populate Map */
      std::vector<int> countGrid(yCells * xCells);
      
      populateMap(cloud_normals, countGrid, xMax, yMax, xMin, yMin, cellResolution, xCells, yCells);

      /* Generate OccupancyGrid Data Vector */
      std::vector<signed char> ocGrid(yCells * xCells);
      genOccupancyGrid(ocGrid, countGrid, xCells * yCells);

      /* Update OccupancyGrid Object */
      updateGrid(grid, cellResolution, xCells, yCells, xMin, yMin, &ocGrid);

      /* Release lock */
      lck.unlock();

      /* Record end time */
      uint32_t esec = ros::Time::now().sec;
      uint32_t ensec = ros::Time::now().nsec;
      std::cout << "Seconds: " << esec - sec << "\nNSeconds: " << ensec - nsec << "\n";

      /* Publish Occupancy Grid */
      pub.publish(grid);
    }
    loop_rate.sleep();
  }
}
