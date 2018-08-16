#include <iostream>
#include <fstream>
#include <stdint.h>
#include <math.h>
//#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
//#include <pcl/cuda/features/normal_3d.h>
//#include <pcl/gpu/features/features.hpp>
//#include <pcl/gpu/containers/initialization.h>
#include <pcl/common/impl/io.hpp>
//#include <data_source.hpp>
#include <pcl/search/search.h>
//#include <pcl/gpu/octree/device_format.hpp>
std::vector<pcl::PointXYZ> downloaded;
double threshold_z1 = 0;  // should add a function to de terimnine the ground automatically
double averange_height = -2; //0.7
int diffThresh = 4;       // the value above certain height - the value under certain height
int greenThresh;          // the thereshold of "may we can pass"
int orangeThresh = 20;         // the thereshold of "high obstacle"
int redThresh = 40;            // the thereshold of "very high obstacle"
int wallThresh =10;           // the thereshold of "can't pass obstacle"
float cutoff_height =4.5;    //0.18 div float cutoff_height =4.5;
/* Define the two point cloud types used in this code */
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud1;
std::vector<std::vector<double> > gridheight;
/* Global */

PointCloud1 currentpc1;
PointCloud1::Ptr currentpc (new PointCloud1);

//currentpc = &currentpc1;

//boost::shared_ptr<const PointCloud<PointT> PointCloud::ConstPtr
PointCloud::ConstPtr currentPC;
bool newPointCloud = false;
bool reconfig = false;
//using namespace cv ;

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
// -----Use GPU to calculate normal---------------
// -----------------------------------------------
/*
void gpuNormal( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){
  std::cout<< "   Running "<<std::endl;
  std::vector<int> sizes;  //  should find what is that
  int max_nn_size;         //how to get the value
  const static int k = 32;
/////////////////
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  std::vector< std::vector<int> > neighbors_all1;
  kdtree->setInputCloud(cloud_filtered);

  size_t cloud_size = cloud_filtered->points.size();

  std::vector<float> dists;
  neighbors_all1.resize(cloud_size);
  for(size_t i = 0; i < cloud_size; ++i)
  {
    kdtree->nearestKSearch(cloud_filtered->points[i], k, neighbors_all1[i], dists);
    sizes.push_back((int)neighbors_all1[i].size());
  }
  max_nn_size = *max_element(sizes.begin(), sizes.end());
  /////////////////////
  pcl::gpu::NormalEstimation::PointCloud cloud;

  cloud.upload(cloud_filtered->points);

  std::vector<int> neighbors_all(max_nn_size * cloud.size());
  pcl::gpu::NeighborIndices indices;

  indices.upload(neighbors_all, sizes, max_nn_size);

  pcl::gpu::NormalEstimation::Normals normals;
  pcl::gpu::NormalEstimation::computeNormals(cloud, indices, normals);
  std::cout<<""<<max_nn_size<<std::endl;
  pcl::gpu::NormalEstimation::flipNormalTowardsViewpoint(cloud, 0.f, 0.f, 0.f, normals);

  normals.download(downloaded);
  for(size_t i = 0; i < downloaded.size(); ++i)
  {

    //std::cout<<downloaded[i].data[3]<<downloaded[i].data[1]<<downloaded[i].data[2]<<std::endl;
  }
  std::cout<<" downloaded "<<std::endl;
}*/
// ------------------------------------------------------
// -----  Resize the size of  gridheight bolock     -----
// ------------------------------------------------------
void PrintIntegerVector(std::vector<std::vector<int> > vector, int r, int c, std::string path){
    std::ofstream file;
    std::string file_path = "/home/echo/桌面/vtk/";
    file_path += path;
    file.open(file_path.c_str());
    for(int i = 0; i < r; i++){
        for(int j = 0; j < c; j++){
            file << vector[i][j];
        }
        file << std::endl;
    }
    file.close();
}

// ------------------------------------------------------
// -----  Resize the size of  gridheight bolock     -----
// ------------------------------------------------------
void resizeBlock(int block_x , int block_y){
    gridheight.resize(block_x);
    for(int i = 0; i < block_x ;i++){
        gridheight[i].resize(block_y);
    }
}
// ------------------------------------------------------
// -----  Out put the height o gridheight bolock    -----
// ------------------------------------------------------
void showBLock(int block_x , int block_y){
    float matrix[block_x*block_y];
    for(int i = 0; i < block_x ;i++){
        std::cout<<std::endl;
        for(int j = 0; j <block_y;j++){
            std::cout<<std::fixed<<std::setprecision(2)<<gridheight[i][j]<<"  ";
            matrix[i*block_y+j]=gridheight[i][j];
        }
   }
    
    cv::Mat image=cv::Mat(block_x, block_y, CV_64FC1, matrix);
    cv::Mat gaussian;
    cv::GaussianBlur(image, gaussian, cv::Size(5, 5), 0, 0);      //This part may cause the break of system
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", gaussian);
    cv::waitKey(0);
    cv::destroyWindow("Display window");
    
}

// ------------------------------------------------------
// -----  Calculate the averange Z height of block  -----
// ------------------------------------------------------
void blockzheight(int width, int height,int minx, int miny, NormalCloud::Ptr cloud_normals){

    int numBlockx =20, numBlocky = 20;
    int blocksizex = width / numBlockx ;
    int blocksizey = height / numBlocky;

    //set the size
    resizeBlock(numBlockx,numBlocky);

    for (int i = 0; i < currentPC->size(); i++)
   {
     int x = currentPC->points[i].x;
     int y = currentPC->points[i].y;
     //calculate the block area
     int blockx = (x-minx) / blocksizex;
     int blocky = (y-miny) / blocksizey;

     double z1 = currentPC->points[i].z;
     std::vector<int> times;
     times.resize(numBlockx * numBlocky + numBlocky);

      double z = cloud_normals->points[i].normal_z;
      double phi = 57.3*acos(fabs(z));

         if ( phi < 5 )
         {
          double z_temp = gridheight[blockx][blocky];
          int u = blockx * blocky + blocky;
          times[u] ++;
          gridheight[blockx][blocky] = (z1/times[u] + z_temp * (times[u]-1)/times[u]);
         }
   }
    showBLock(numBlockx,numBlocky);
}
// -----------------------------------------------
// -----  To get the Z block                 -----
// -----------------------------------------------
void blockSeg(int xblocksize, int yblocksize, int xtotal, int ytotal, std::vector<std::vector<double> > gridheight ){
    int sizex = xtotal/xblocksize;
    int sizey = ytotal/yblocksize;

}
// -----------------------------------------------
// -----  To find the ground automatcally    -----
// -----------------------------------------------
void groundZfinder(int xMax, int yMax, int xMin, int yMin, NormalCloud::Ptr cloud_normals){
  std::vector<std::vector<double> > groundDifReg; // to save the averange z aixs height 
  std::vector<std::vector<double> > gridheight;
 // double deviation = param.deviation;
  int xrange = xMax - xMin;
  int yrange = yMax - yMin;

  int blockx = 3;
  int blocky = 3;
// resize the block
  gridheight.resize(blockx);
  for(int j =0; j < blockx; j++)
 {
   gridheight[j].resize(blocky);
 }

  int rows =2;
  double averangeZheight;
  int counter_z = 0;
  int MapArea = (xMax-xMin)*(yMax-yMin);
  double matrix[MapArea];

  groundDifReg.resize(rows);
     for(int j =0; j < rows; j++)
    {
      groundDifReg[j].resize(MapArea); //the size is rows*Map area the [0] saves the times appears
    }
  groundDifReg.clear();
  
   for (size_t i = 0; i < currentPC->size(); i++) 
  {
     
    int x = currentPC->points[i].x;
    int y = currentPC->points[i].y;
    int xCell = (x - xMin) ;
    int yCell = (y - yMin) ;

    double z = cloud_normals->points[i].normal_z;
    double z_axis = currentPC->points[i].z;  // get the autitute of current pose
    double phi = 57.3*acos(fabs(z));

    if ( phi < 5 ){

    double z_temp = 0;
    groundDifReg[0][yCell*xrange+xCell]++;
    z_temp = groundDifReg[1][yCell*xrange+xCell];

    groundDifReg[1][yCell*xrange+xCell] = (z_axis * (1/groundDifReg[0][yCell*xrange+xCell]) + z_temp * (groundDifReg[0][yCell*xrange+xCell]-1)/groundDifReg[0][yCell*xrange+xCell] );
   // std::cout<<yCell*xrange+xCell<< " pose   number:   "<<groundDifReg[0][yCell*xrange+xCell]<<
   // "  value:  "<<groundDifReg[1][yCell*xrange+xCell]<<"   z_axis: "<<z_axis<<std::endl;
    }
  }

for(int u=0; u<xrange; u++)
{
  for(int v=0; v<yrange; v++)
  {
    //matrix[u*yrange+v]= groundDifReg[1][u*yrange+v] + 5; //abs!!!!!!!!
    matrix[u*yrange+v]= groundDifReg[1][u*yrange+v]; //abs!!!!!!!!
    if(!matrix[u*yrange+v] == 0){
        counter_z++;
    averangeZheight = (matrix[u*yrange+v] + averangeZheight*(counter_z-1))/counter_z ; // may the value is too large for this thing
    }
  }
}
threshold_z1=averangeZheight;
std::cout<<"averangeZheight :"<<averangeZheight<<std::endl;

// blockSeg( blockx, blocky, xrange, yrange, gridheight);
/*
cv::Mat image=cv::Mat(yrange, xrange, CV_64FC1, matrix);
cv::Mat gaussian;
cv::GaussianBlur(image, gaussian, cv::Size(5, 5), 0, 0);      //This part may cause the break of system
cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
cv::imshow( "Display window", gaussian);
cv::waitKey(0);
cv::destroyWindow("Display window");
*/

}

// the out put is the range block1 (x1-x2,y1-y2)-> the thereshold of Axis z


// ------------------------------------------------------
// -----Update current PointCloud if msg is received-----
// ------------------------------------------------------
void callback(const PointCloud::ConstPtr& msg) {
  boost::unique_lock<boost::mutex>(mutex);
  currentPC = msg;
   pcl::copyPointCloud(*currentPC, *currentpc);

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
    
    int the_size_of_map;
    the_size_of_map = (xMax-xMin)*(yMax-yMin)/(cellResolution*cellResolution);

    int conditions=2;
    map_n.clear();
    map_n.resize(conditions);

    std::cout<<" the averanger z axis height is :"<< threshold_z1<< std::endl;

    for(int j =0; j < conditions; j++)
    {
      map_n[j].resize(the_size_of_map); // whether it is not big enough to put the point cloud in？currentPC->size()/cellResolution
    }

    std::cout<< "test point 1"<<std::endl;
    //int x1 = currentPC->size()/(cellResolution*cellResolution);
  for (size_t i = 0; i < currentPC->size(); i++) 
  {
    double x = currentPC->points[i].x;
    double y = currentPC->points[i].y;
    double z = cloud_normals->points[i].normal_z;
    double z_axis = currentPC->points[i].z;

    double phi = acos(fabs(z));
    int xCell, yCell;
   // std::cout<< i <<" i,  the_size_.of_map: "<<the_size_of_map<<"  the place to write in map_n "<<yCell * xCells + xCell<<std::endl;
    if ( 1) { //TODO implement cutoff height!!!       z_axis < threshold_z1 + cutoff_height
      xCell = (int) ((x - xMin) / cellResolution);
      yCell = (int) ((y - yMin) / cellResolution);


      if (phi > deviation) 
      {
        if(z_axis <  threshold_z1 + 1)               //TODO this part is to calculate the height of ground               threshold_z1 +
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
        map[yCell * xCells + xCell]=1;
       // map[yCell * xCells + xCell]--;
       }
    }
  }
    std::cout<< "test point 2"<<std::endl;
      for(int m =0; m < currentPC->size(); m++)
    {
        double x1 = currentPC->points[m].x;
        double y1 = currentPC->points[m].y;
        int xCell1 = (int) ((x1 - xMin) / cellResolution);
        int yCell1 = (int) ((y1 - yMin) / cellResolution);
        //std::cout<<" current m:"<<m<<std::endl;
      if(map_n[0][yCell1 * xCells + xCell1] > map_n[1][yCell1 * xCells + xCell1] && map_n[0][yCell1 * xCells + xCell1] >= 15) //TODO  here is calculate the greeen block
      {
          //TODO m is not poper with yCell * xCells + xCell
      map[yCell1 * xCells + xCell1] = -1;
          //std::cout<< xCells*yCells<<"size , current m:"<<m<<std::endl;
      }
    }
    std::cout<< "test point 3"<<std::endl;
    //std::cout<<"  populate map is finished :"<<  std::endl;
  }


//double threshold_z1 = 0;  // should add a function to de terimnine the ground automatically
//double averange_height = -2; //0.7
//int diffThresh = 4;       // the value above certain height - the value under certain height
//int greenThresh;          // the thereshold of "may we can pass"
//int orangeThresh = 20;         // the thereshold of "high obstacle"
//int redThresh = 40;            // the thereshold of "very high obstacle"
//int wallThresh =10;           // the thereshold of "can't pass obstacle"
//float cutoff_height =4.5;    //0.18 div float cutoff_height =4.5;
// ---------------------------------
// -----Generate Occupancy Grid-----
// -------------------else if (countGrid[i] < 40&&countGrid[i]>20) {

void genOccupancyGrid(std::vector<signed char> &ocGrid, std::vector<int> &countGrid, int size , int xcells) {
  int buf = param.buffer;

  for (int i = 0; i < size; i++) {
    if (countGrid[i] < 4 && countGrid[i]>0)   //10 is not changeds
    {
      ocGrid[i] = 0;
    } else if (countGrid[i] > 10 &&countGrid[i] <= 4)
    {
     // std::cout<<countGrid[i]<<"    ";
      ocGrid[i] = 100;
    } else if (countGrid[i] == 0) 
    {
        if(countGrid[i+1] == 0 && countGrid[i-1] == 0){
            ocGrid[i] = -1;//2018.1.15 not 100 used to be -1
        } else{
            ocGrid[i] = 0;
        }

    } else if (countGrid[i] < 20 && countGrid[i] >= 10)
    {
      ocGrid[i] = 100; // TODO Should be -1 200
    } else if (countGrid[i] >= 20)
    {
      ocGrid[i] = 100; // TODO Should be -1 150
    } else if (countGrid[i] = -1) //2018.1.15 not 100 used to be -1
    {
        if(countGrid[i+1] == -1 || countGrid[abs(i-1)] == -1 || countGrid[abs(i-xcells)]== -1 || countGrid[i+xcells] == -1){
            ocGrid[i] = 100;//125
        }else{
            ocGrid[i] = 0; 
        }
       // ocGrid[i] = 100;
    }

  }
    std::cout<<" genOccupancyGrid finshed:"<< std::endl;
}

// --------------
// -----Main-----
// --------------
int main(int argc, char** argv) {
  /* Initialize ROS */
  ros::init(argc, argv, "cloud_to_map_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/passthrough", 1, callback);
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

      //gpuNormal(currentpc);
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

      /* Detect the ground*/
      groundZfinder(xMax, yMax, xMin, yMin,cloud_normals);
      blockzheight(xMax-xMin, yMax-yMin, xMin,yMin, cloud_normals);

      populateMap(cloud_normals, countGrid, xMax, yMax, xMin, yMin, cellResolution, xCells, yCells);
        std::cout<<" Ocgriding :"<< std::endl;
      /* Generate OccupancyGrid Data Vector */

      std::vector<signed char> ocGrid(yCells * xCells);
        std::cout<<" genOccupancyGrid  start :"<< std::endl;
      genOccupancyGrid(ocGrid, countGrid, xCells * yCells, xCells);
        std::cout<<" genOccupancyGrid :"<< std::endl;
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
