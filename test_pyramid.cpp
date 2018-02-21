#include "GSHOTPyramid.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <cstdlib>
#include <sys/timeb.h>


using namespace FFLD;
using namespace std;
using namespace Eigen;




//Read point cloud from a path
int
readPointCloud(std::string object_path,  boost::shared_ptr<pcl::PointCloud<PointType> > point_cloud)
{
    std::string extension = boost::filesystem::extension(object_path);
    if (extension == ".pcd" || extension == ".PCD")
    {
        if (pcl::io::loadPCDFile(object_path.c_str() , *point_cloud) == -1)
        {
            std::cout << "\n Cloud reading failed." << std::endl;
            return (-1);
        }
    }
    else if (extension == ".ply" || extension == ".PLY")
    {
        if (pcl::io::loadPLYFile(object_path , *point_cloud) == -1)
        {
            std::cout << "\n Cloud reading failed." << std::endl;
            return (-1);
        }
    }
    else
    {
        std::cout << "\n file extension is not correct. Syntax is: ./main <path/file_name.pcd> [--nogui] or ./main <path/file_name.ply>" << std::endl;
        return -1;
    }
    return 1;
}


int
main (int argc, char *argv[]){
    
    //Testing parameters
    int octaves = 3;//Etage
    int intervals = 5; // nombre de section à chaque octa
    float starting_resolution = 0.01;
    float starting_kp_reso = 0.2;
    float starting_descriptor_radius = 0.4;
    
    //Constructor Tensor<data_type, rank>(size0, size1, ...)
    Tensor<float, 3> t_3d(2, 3, 4);
    
 

   
  

        pcl::PointCloud<PointType>::Ptr object(new pcl::PointCloud<PointType>);
        std::string cloud_path;
        if(pcl::console::parse_argument(argc, argv, "-cloud", cloud_path) == -1)
        {
            std::cerr<<"Please specify the cloud"<<std::endl;
            return -1;
        }
        
        if (readPointCloud(cloud_path,  object)==-1)
        return;
        
        boost::filesystem::path p(cloud_path);
        std::string filename =  p.filename().c_str();

        GSHOTPyramid* pyr(new GSHOTPyramid(object, octaves, intervals, starting_resolution, starting_kp_reso, starting_descriptor_radius));
    
    std::cout << "\n ####TEST " << std::endl;
    //pyr->test();
    
    std::cout << "\n ####END TEST " << std::endl;
    
        pyr->toString();
    
        /*
        std::cout << std::endl;
        std::cout << "Time elapsed for building the feature pyramid : " << milliSecondsElapsed << " ms." << std::endl;
        
        std::vector<double> feature_vector_final(352);
        std::vector<double> data_tmp,feature_vector_final_scaled;
        std::fill(feature_vector_final.begin(), feature_vector_final.end(), 0);
        
        // Pyramid validation using histograms
        for(unsigned int w=0;w<pyr->get_height();w++){
            pcl::PointCloud<DescriptorType> descr_cloud = pyr->get_descriptors_layer(w);
            std::vector<double> feature_vector(352);
            for(unsigned int i=0;i<descr_cloud.size();i++){
                for(unsigned int j=0;j<352;j++) {
                    if (!pcl_isfinite (descr_cloud[i].descriptor[j])) // Replacing NaNs by zeros
                    {
                        continue;
                    }
                    else {
                        feature_vector[j] += descr_cloud[i].descriptor[j];
                    }
                }
            }
            
            float max = 0;
            for(unsigned int i=0;i<352;i++){
                if(feature_vector[i]>max)
                max = feature_vector[i];
            }

            
            //Sum element wise all descriptor
            std::transform (feature_vector_final.begin(), feature_vector_final.end(), feature_vector.begin(), feature_vector_final.begin(), std::plus<double>());
            
        
        }
        double max = 0;
        for(unsigned int i=0;i<352;i++){
            if(feature_vector_final[i]>max)
            max = feature_vector_final[i];
        }
        
        return 0;
    */
}

