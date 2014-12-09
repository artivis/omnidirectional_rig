/* \author Geoffrey Biggs */

#ifndef PCL_VIEW_H
#define PCL_VIEW_H


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/visualization/eigen.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("sphere"));
  viewer->initCameraParameters();

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

  int v1(1);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "right", v1);

  int v2(2);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->createViewPortCamera(v2);
  viewer->setBackgroundColor (0, 0, 0, v2);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "left", v2);

  viewer->setCameraPosition(1,0,0,-1,0,0,0,-0,1, v1);
  viewer->setCameraPosition(1,0,0,-1,0,0,0,-0,1, v2); //try to get same image as 1st cam

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "right", v1);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "left", v2);

  viewer->setCameraClipDistances(0.01,3,v1);
  viewer->setCameraClipDistances(0.01,3,v2);

  return (viewer);
}

#endif // PCL_VIEW_H
