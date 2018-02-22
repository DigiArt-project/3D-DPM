
#ifndef FFLD_POINTCLOUD_H
#define FFLD_POINTCLOUD_H

//PCL Header
#include <stdio.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include "pcl/features/normal_3d.h"
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include  <pcl/filters/uniform_sampling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

// Boost headers
#include <boost/filesystem.hpp>
#include <boost/timer/timer.hpp>

#include <iosfwd>
#include <string>
#include <vector>
#include <stdint.h>
#include "typedefs.hpp"
#include <Eigen/Core>

namespace FFLD
{

class PointCloudData
{
public:
    enum typeFilename { ply, obj, pcd };

    /** CONSTRUCTORS **/
    
    /// Constructs an empty pointCloud. An empty pointCloud has zero size.
    PointCloudData();
    
    /// Constructs and initialize a point cloud object with the given @p filename
    /// @note The returned point cloud might be empty if any of the parameters is incorrect.
    PointCloudData(const std::string & filename);
    
    /** GETTERS AND SETTERS **/
    
    //Returns the size of the point cloud.
    /// @note The returned size is width * height
    int getSize() const;
    
    /// Returns the width of the point cloud.
    int getWidth() const;
    
    /// Returns the height of the point cloud. If the result is one, it means that the point cloud is unorganized
    int getHeight() const;
    
    //Returns the centroid of the point cloud
    Eigen::Vector4f getCentroid() const;
    
    /// Returns whether the point cloud is empty. An empty point cloud has zero size.
    bool isEmpty() const;
    
    //Return wheter the point cloud is organized or not
    bool isOrganized() const;
    
    // Return the resolution of the point cloud
    double getResolution() const;
    
    /** FILTERING **/
    
    //perform a simple filtering along z axis – that is, cut off values that are either inside or outside a given user range @p min_depth and @p max_depth
    void thresholdDepth (float min_depth, float max_depth);
    
    //downsample the point cloud – that is, reduce the number of points – a point cloud dataset, using a voxelized grid approach and a given @p leaf_size
    void downsample (float leaf_size);
   
    //Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors @p radius and @p min_neighbors
    void removeOutliers (float radius, int min_neighbors);
    
    //Apply a series of filters (threshold depth, downsample, and remove outliers) to the point cloud
    void applyFilters (float min_depth, float max_depth, float leaf_size, float radius,
                                float min_neighbors);
    
    //Apply an uniform sampling to the point cloud
    void uniformSampling(float radiusSearch);
    
    //Filter the point cloud so that the entire point cloud is fitting the given boundary corresponding a to a box
    void bounding_box(const float & pLimitMinX,const float & pLimitMaxX,const float & pLimitMinY,const float & pLimitMaxY,
                      const float & pLimitMinZ,
                      const float & pLimitMaxZx
                      );
    
    /** FEATURES **/
    
    void computeCentroid();
    
    //Estimating the normals of the point cloud given a specific @p radius
    SurfaceNormalsPtr estimateSurfaceNormals(float radius = 0.01f);

    //Compute the resolution of the point cloud
    void compute_resolution();
    
   /** VISUALIZATION **/
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizeCloud ();
    
    
    /** OTHER **/
    
    //Normalize the point cloud in terme of scale and translation
    void normalizePointCloud();
    
    //Translate the point cloud to the origin
    void translateToOrigin();
    
    //Display Point cloud information
    void toString();
    
    /// Saves the point cloud to a pcd file with the given @p filename and @p binary format.
    void savePointCloud(const std::string & filename, bool binary = false);
    
	
private:
    Eigen::Vector4f m_centroid;
    pcl::PointCloud<NormalType>::Ptr m_normals;
    pcl::PointCloud<PointType>::Ptr m_cloud;
    bool m_isOrganized;
    int m_size;
    int m_height;
    int m_width;
    double m_resolution;
};

/// Serializes an image to a stream.
std::ostream & operator<<(std::ostream & os, const PointCloudData & pointCloud);

}

#endif
