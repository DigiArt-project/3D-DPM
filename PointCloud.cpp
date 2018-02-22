#include "PointCloud.h"

using namespace Eigen;
using namespace FFLD;
using namespace std;

/** CONSTRUCTORS **/
#pragma mark -  CONSTRUCTORS METHODS

PointCloudData::PointCloudData(): m_size(0),m_width(0),m_height(0),m_resolution(0){
    SurfaceNormalsPtr normals(new  SurfaceNormals());
    this->m_normals = normals;
    PointCloudPtr cloud(new PointCloudT());
    this->m_cloud = cloud;

}

//Read point cloud from a path
PointCloudData::PointCloudData(const std::string & filename)
{
    std::string extension = boost::filesystem::extension(filename);
    if (extension == ".pcd" || extension == ".PCD")
    {
        if (pcl::io::loadPCDFile(filename.c_str() , *m_cloud) == -1)
        {
            std::cout << "\n Cloud reading failed." << std::endl;
            return;
        }
    }
    else if (extension == ".ply" || extension == ".PLY")
    {
        if (pcl::io::loadPLYFile(filename , *m_cloud) == -1)
        {
            std::cout << "\n Cloud reading failed." << std::endl;
            return;
        }
    }
    else if (extension == ".obj" || extension == ".OBJ")
    {
        if (pcl::io::loadOBJFile(filename , *m_cloud) == -1)
        {
            std::cout << "\n Cloud reading failed." << std::endl;
            return;
        }
    }
    else
    {
        std::cout << "\n file extension is not correct." << std::endl;
        return;
    }
    
    this->m_size = m_cloud->size();
    this->m_width = m_cloud->width;
    this->m_height = m_cloud->height;
    
    if (this->m_height == 1){
        this->m_isOrganized = true;
    }else{
        this->m_isOrganized = false;
    }
    
    normalizePointCloud();
    computeCentroid();
}

/** FILTERING **/
#pragma mark -  FILTERING METHODS

/* Use a PassThrough filter to remove points with depth values that are too large or too small */
void PointCloudData::thresholdDepth (float min_depth, float max_depth)
{
    pcl::PassThrough<PointType> pass_through;
    pass_through.setInputCloud (m_cloud);
    pass_through.setFilterFieldName ("z");
    pass_through.setFilterLimits (min_depth, max_depth);
    PointCloudPtr thresholded (new PointCloudT);
    pass_through.filter (*m_cloud);
}

void PointCloudData::bounding_box
(
 const float & pLimitMinX,
 const float & pLimitMaxX,
 const float & pLimitMinY,
 const float & pLimitMaxY,
 const float & pLimitMinZ,
 const float & pLimitMaxZ
 )
{
    
    PointCloudPtr filtered_cloud_z(new PointCloudT());
    PointCloudPtr filtered_cloud_x(new PointCloudT());
    
    pcl::PassThrough<PointType> pass_through;
    pass_through.setInputCloud(m_cloud);
    pass_through.setKeepOrganized(true);
    pass_through.setFilterFieldName("z");
    pass_through.setFilterLimits(pLimitMinZ, pLimitMaxZ);
    pass_through.filter(*filtered_cloud_z);
    
    pass_through.setInputCloud(filtered_cloud_z);
    pass_through.setFilterFieldName("x");
    pass_through.setFilterLimits(pLimitMinX, pLimitMaxX);
    pass_through.filter(*filtered_cloud_x);
    
    pass_through.setInputCloud(filtered_cloud_x);
    pass_through.setFilterFieldName("y");
    pass_through.setFilterLimits(pLimitMinY, pLimitMaxY);
    pass_through.filter(*m_cloud);
    
}

/* Use a VoxelGrid filter to reduce the number of points */
void PointCloudData::downsample (float leaf_size)
{
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud (m_cloud);
    voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    PointCloudPtr downsampled (new PointCloudT);
    voxel_grid.filter (*m_cloud);

}

/* Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
void PointCloudData::removeOutliers (float radius, int min_neighbors)
{
    pcl::RadiusOutlierRemoval<PointType> radius_outlier_removal;
    radius_outlier_removal.setInputCloud (m_cloud);
    radius_outlier_removal.setRadiusSearch (radius);
    radius_outlier_removal.setMinNeighborsInRadius (min_neighbors);
    PointCloudPtr inliers (new PointCloudT);
    radius_outlier_removal.filter (*m_cloud);
}

/* Apply a series of filters (threshold depth, downsample, and remove outliers) */
void PointCloudData::applyFilters (float min_depth, float max_depth, float leaf_size, float radius,float min_neighbors)
{
    PointCloudPtr filtered;
    thresholdDepth (min_depth, max_depth);
    downsample (leaf_size);
    removeOutliers (radius, min_neighbors);

}

void PointCloudData::uniformSampling(float radiusSearch){
    // Uniform sampling object.
    pcl::UniformSampling<PointType> filter;
    filter.setInputCloud(m_cloud);
    // We set the size of every voxel to be 1x1x1cm
    // (only one point per every cubic centimeter will survive).
    filter.setRadiusSearch(radiusSearch);
    // We need an additional object to store the indices of surviving points.
    pcl::PointCloud<int> keypointIndicesInput;
    PointCloudPtr output (new PointCloudT);
    filter.filter(*m_cloud);
    
}

/** FEATURES **/
#pragma mark -  FEATURES METHODS

SurfaceNormalsPtr PointCloudData::estimateSurfaceNormals(float radius){
    pcl::NormalEstimation<PointType, NormalType> normal_estimation;
    normal_estimation.setSearchMethod (pcl::search::Search<PointType>::Ptr (new pcl::search::KdTree<PointType>));
    normal_estimation.setRadiusSearch (radius);
    normal_estimation.setInputCloud (m_cloud);
    SurfaceNormalsPtr normals (new SurfaceNormals);
    normal_estimation.compute (*normals);
    
    return (normals);
}

void PointCloudData::compute_resolution(){
    double resolution = 0.0;
    int points = 0;
    int nres;
    
    std::vector<int> indices(2);
    std::vector<float> sqrDistances(2);
    pcl::search::KdTree<PointType> kdtree;
    kdtree.setInputCloud(m_cloud);
    
    for (size_t i = 0; i < m_cloud->size(); ++i)
    {
        if (!pcl_isfinite((*m_cloud)[i].x))
            continue;
        
        nres = kdtree.nearestKSearch(i, 2, indices, sqrDistances);
        
        if (nres == 2)
        {
            resolution += sqrt(sqrDistances[1]);
            ++points;
        }
    }
    
    if (points != 0)
        resolution /= points;
    
    std::cout << "Cloud resolution is " << resolution << "\n";
    
    m_resolution = resolution;
}


void PointCloudData::computeCentroid(){
    pcl::compute3DCentroid (*m_cloud, m_centroid);
}


/** VISUALIZATION **/
#pragma mark -  VISUALIZATION METHODS

boost::shared_ptr<pcl::visualization::PCLVisualizer> PointCloudData::visualizeCloud ()
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D cloud Viewer"));
    viewer->setBackgroundColor (1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> object_cloud_color_handler (m_cloud, 255, 0, 0);
    viewer->setSize(2000, 2000);
    viewer->addPointCloud<PointType> (m_cloud,object_cloud_color_handler, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while(!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
    
    return (viewer);
}
void PointCloudData::savePointCloud(const std::string & filename,bool binary){
    std::string extension = boost::filesystem::extension(filename);
    if (extension == ".pcd" || extension == ".PCD")
    {
        if (binary){
            pcl::io::savePCDFileASCII (filename, *m_cloud);
        }else {
            pcl::io::savePCDFileBinary (filename, *m_cloud);
        }
    }
    else {
        std::cerr << "[Saving Point Cloud Error] Extension is not valid. Need .pcd" << std::endl;
    }
   
    
}

/** VISUALIZATION **/
#pragma mark -  GETTERS AND SETTERS METHODS


Eigen::Vector4f PointCloudData::getCentroid() const{
    
    return m_centroid;
}
double PointCloudData::getResolution() const
{
    return m_resolution;
}

int PointCloudData::getSize() const
{
    return m_size;
}

int PointCloudData::getWidth() const
{
    return m_width;
}

int PointCloudData::getHeight() const
{
    return m_height;
}

bool PointCloudData::isEmpty() const
{
    return (getSize() <= 0);
}

bool PointCloudData::isOrganized() const
{
    return m_isOrganized;
}


/** OTHERS **/
#pragma mark -  OTHERS METHODS

void PointCloudData::translateToOrigin(){
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // Define a translation
    transform.translation() << -m_centroid[0], -m_centroid[1], -m_centroid[2];
    // Executing the transformation
    PointCloudPtr transformed_cloud (new PointCloudT());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*m_cloud, *m_cloud, transform);
    
}


void PointCloudData::normalizePointCloud(){
    
    PointType centroid;
    pcl::computeCentroid(*m_cloud, centroid);
    
    std::vector<int> indices;
    std::vector<float> distances;
    
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud( m_cloud, NULL);
    kdtree.nearestKSearch( centroid, m_cloud->size(), indices, distances);
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
    transform.translation() << -centroid.x/sqrt(distances.back()), -centroid.y/sqrt(distances.back()), -centroid.z/sqrt(distances.back());
    transform.scale(1/sqrt(distances.back()));
    
    pcl::transformPointCloud (*m_cloud, *m_cloud, transform);
    
}

void PointCloudData::toString()
{
    
    std::cout<<"Point Cloud Information :"<<std::endl<<std::endl;
    std::cout<<"Width: "<<this->getWidth()<<std::endl;
    std::cout<<"Height: "<<this->getHeight()<<std::endl;
    std::cout<<"Total Size: "<<this->getSize()<<std::endl;
    std::cout<<"Resolution: "<<this->getResolution()<<std::endl;
    std::cout<<"Centroid: ("<<this->getCentroid().x()<< "," << this->getCentroid().y() << "," << this->getCentroid().z() << ")" << std::endl;
}

//TODO
std::ostream & FFLD::operator<<(std::ostream & os, const PointCloudData & pointCloud){
    
    os << pointCloud.getWidth() << ' ' << pointCloud.getHeight() << ' ' << pointCloud.getSize() << ' ';
    
    return os << endl;
}
