// Written by Fisichella Thomas
// Date 25/05/2018

#ifndef typedefs_h
#define typedefs_h

using namespace std;


/* Define some custom types to make the rest of our code easier to read */
typedef pcl::ReferenceFrame RFType;

// Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGB points
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudT;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointType>::ConstPtr PointCloudConstPtr;
//Vector of point cloud
typedef std::vector<PointCloudPtr> cloudVectorT;

// Define "SurfaceNormals" to be a pcl::PointCloud of pcl::Normal points
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> SurfaceNormals;
typedef pcl::PointCloud<NormalType>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalType>::ConstPtr SurfaceNormalsConstPtr;

// Define "Descriptors" to be a pcl::PointCloud of SHOT points. Can change
//typedef pcl::SHOT352 LocalDescriptor;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<DescriptorType> Descriptors;
typedef pcl::PointCloud<DescriptorType>::Ptr DescriptorsPtr;
typedef pcl::PointCloud<DescriptorType>::ConstPtr DescriptorsConstPtr;

//namespace FFLD
//{
//    //Read point cloud from a path
//    static int readPointCloud(std::string object_path, PointCloudPtr point_cloud){
//        std::string extension = boost::filesystem::extension(object_path);
//        if (extension == ".pcd" || extension == ".PCD")
//        {
//            if (pcl::io::loadPCDFile(object_path.c_str() , *point_cloud) == -1)
//            {
//                std::cout << "\n Cloud reading failed." << std::endl;
//                return (-1);
//            }
//        }
//        else if (extension == ".ply" || extension == ".PLY")
//        {
//            if (pcl::io::loadPLYFile(object_path.c_str() , *point_cloud) == -1)
//            {
//                std::cout << "\n Cloud reading failed." << std::endl;
//                return (-1);
//            }
//        }
//        else
//        {
//            std::cout << "\n file extension is not correct." << std::endl;
//            return -1;
//        }
//        return 1;
//    }

//}



#endif /* typedefs_h */
