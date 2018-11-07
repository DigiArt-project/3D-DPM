// Written by Fisichella Thomas
// Date 25/05/2018

#ifndef FFLD_GSHOTPYRAMID_H
#define FFLD_GSHOTPYRAMID_H

//EIGEN
#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>

#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//Other
#include "typedefs.h"
#include "tensor3d.h"
#include "Rectangle.h"
#include <boost/filesystem.hpp>
#include <omp.h>


namespace FFLD
{

    template< typename T1, typename T2, typename T3>
    struct triple : std::tuple<T1, T2, T3>{
        triple(){
            first = 0;
            second = 0;
            third = 0;
        }
        triple( std::tuple<T1, T2, T3> t){
            first = std::get<0>(t);
            second = std::get<1>(t);
            third = std::get<2>(t);
        }
        triple( T1 t1, T2 t2, T3 t3){
            first = t1;
            second = t2;
            third = t3;
        }

        friend std::ostream & operator<<(std::ostream & os, const triple & tri){
            os << "[" << tri.first << ";" << tri.second << ";" << tri.third << "]";
            return os;
        }

        T1 first;
        T2 second;
        T3 third;
    };

    class GSHOTPyramid
    {
        public:
        /** DEFINITION OF TYPES **/
        
        /// Number of SHOT features (guaranteed to be even). Fixed at compile time for both ease of use
        /// and optimal performance.
        static const int DescriptorSize = 352;
        typedef float Scalar;

        
        /// Type of a matrix.
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix;

        /// Type of a pyramid level cell (fixed-size array of length NbFeatures).
        typedef Eigen::Array<Scalar, DescriptorSize, 1> Cell;

        /// Type of a pyramid level (matrix of cells).
        typedef Tensor3D<Cell> Level;
        
                
        /** CONSTRUCTORS **/
        
        /// Constructs an empty pyramid. An empty pyramid has no level.
        GSHOTPyramid();

        // Copy constructor
        GSHOTPyramid(const GSHOTPyramid&);
        
        /// Constructs a pyramid from the PointCloud of a Scene.
        /// @param[in] input The PointCloud data
        /// @param[in] pad Amount of horizontal, vertical and depth zero padding (in cells).
        /// @param[in] interval Number of levels per octave in the pyramid.
        /// @note The amount of padding and the interval should be at least 1.

        GSHOTPyramid(Eigen::Vector3i filterSizes, int interval = 1,
                     float starting_resolution = 0.1, int nbOctave = 2);


        void createPyramid(const PointCloudPtr input, int thresh = 400);

        PointCloudPtr createPyramid(const PointCloudPtr input, vector<Eigen::Vector3i> colors, int thresh = 400);
        
        // Destructor
//        ~GSHOTPyramid();

        /** GETTERS AND SETTER **/
        
        /// Returns whether the pyramid is empty. An empty pyramid has no level.
        bool empty() const;

        Eigen::Vector3i pad() const;
        
        /// Returns the number of levels per octave in the pyramid.
        int interval() const;
        
        /// Returns the pyramid levels.
        /// @note Scales are given by the following formula: 2^(1 - @c index / @c interval).
        const vector<vector<Level> > &levels() const;

        const std::vector<float> & resolutions() const;
        
        /** OTHERS **/
    
        
        /// Returns the convolutions of the pyramid with a filter.
        /// @param[in] filter Filter.
        /// @param[out] convolutions Convolution of each level.
        void convolve(const Level & filter, vector<vector<Tensor3DF> > &convolutions) const;

        void sumConvolve(const Level & filter, vector<Tensor3DF >& convolutions) const;

        /// Maps a const pyramid level to a simple const matrix (useful to apply standard matrix
        /// operations to it).
        /// @note The size of the matrix will be rows x (cols * NbFeatures).
        /// Should be moved in another class
        static Tensor3DF TensorMap(Level level);
        
        /// Return the mean resolution of the cloud
        static double computeCloudResolution (PointCloudConstPtr cloud);
        
//        private:

        static Eigen::Matrix4f getNormalizeTransform(float* orientationFrom, float* orientationTo,
                                                     const Eigen::Vector3f origin,
                                                     const Eigen::Vector3f translation = Eigen::Vector3f(0,0,0));
        
        std::vector<float> minMaxScaler(std::vector<float> data);

        // Method for computing feature spaces. May have different implementations depending on the descriptor used.
        DescriptorsPtr
        compute_descriptor(PointCloudPtr input, PointCloudPtr keypoints, float);
        
        PointCloudPtr computeKeyptsWithThresh(PointCloudPtr cloud, float grid_reso, PointType min, PointType max,
                                              Eigen::Vector3i filterSizes, int thresh);

        // Method creating the keypoint grid using the min/max values of the input
        /*static */PointCloudPtr
        compute_keypoints(float grid_reso, PointType min, PointType max, int index);
        
        // Computes the 2D convolution of a pyramid level with a filter
        static void Convolve(const Level & level, const Level & filter, Tensor3DF & convolution);
        
//        // Number of keypoints per dimension (needed for the sliding box process)
        std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > topology_;//number of points at [lvl]

        Eigen::Vector3i pad_;
        int interval_;
        int nbOctave_;
        // Represent a vector of 3D scene of descriptors computed at different resolution
        //from 0 (original resolution) to n (lowest resolution, last octave)
        std::vector<std::vector<Level> > levels_;

        std::vector<float> resolutions_;


        // The corresponding positions of boxes descriptors in the space for each level
        std::vector<std::vector<PointCloudPtr > > keypoints_;//[lvl][box]
        std::vector<std::vector<Rectangle> > rectangles_;//[lvl][box]

        PointCloudPtr globalKeyPts;
        DescriptorsPtr globalDescriptors;

        Eigen::Vector3i filterSizes_;

        Eigen::Vector3i sceneOffset_;
    };
    
    //Read point cloud from a path
    /*static*/ int readPointCloud(std::string object_path, PointCloudPtr point_cloud);

    /// Serializes a pyramid to a stream.
    std::ostream & operator<<(std::ostream & os, const GSHOTPyramid & pyramid);
    
    /// Unserializes a pyramid from a stream.
    std::istream & operator>>(std::istream & is, GSHOTPyramid & pyramid);
}

#endif

