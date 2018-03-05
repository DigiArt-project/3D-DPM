#ifndef FFLD_GSHOTPYRAMID_H
#define FFLD_GSHOTPYRAMID_H

//EIGEN
#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>

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
#include <vector>

//Other
#include "typedefs.h"

//Subsection = interval ?
//Padding ?
#ifdef _OPENMP
#include <omp.h>
#endif


namespace FFLD
{
    class GSHOTPyramid
    {
        public:
        /** DEFINITION OF TYPES **/
        
        /// Number of HOG features (guaranteed to be even). Fixed at compile time for both ease of use
        /// and optimal performance.
        static const int NbFeatures = 352;
        typedef float Scalar;

        
        /// Type of a matrix.
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix;

        template <typename Type>
        struct Tensor : Eigen::Tensor<Type, 3, Eigen::RowMajor>{
            Tensor( int s1, int s2, int s3) : Eigen::Tensor<Type, 3, Eigen::RowMajor>(s1, s2, s3){}
            //row d'une matrice --> renvoie ligne de la matrice
            
            Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            row( int i) const{
                Eigen::Matrix<Type, this->dimension(0), this->dimension(1), Eigen::RowMajor> res;
                return res.setZero();
            }

            int rows(){
                return *this->dimension(0);
            }
            int cols(){
                return *this->dimension(1);
            }
            int depths(){
                return *this->dimension(2);
            }
            
        };
        
        /// Type of a pyramid level cell (fixed-size array of length NbFeatures).
        typedef Eigen::Array<Scalar, NbFeatures, 1> Cell;
        
        /// Type of a pyramid level (matrix of cells).
        typedef Tensor<Cell> Level;
        
        
        void test() const;
        
        /** CONSTRUCTORS **/
        
        /// Constructs an empty pyramid. An empty pyramid has no level.
        GSHOTPyramid();
        
        // Copy constructor
        GSHOTPyramid(const GSHOTPyramid&);
        
        /// Constructs a pyramid from the JPEGImage of a Scene.
        /// @param[in] input The PointCloud data
        /// @param[in] pad Amount of horizontal, vertical and depth zero padding (in cells).
        /// @param[in] interval Number of levels per octave in the pyramid.
        /// @note The amount of padding and the interval should be at least 1.
        GSHOTPyramid(const pcl::PointCloud<PointType>::Ptr input, Eigen::Vector3i pad, int interval = 5);

        /// Constructs a pyramid from a given point cloud data.
        /// @param[in] input The PointCloud data
        /// @param[in] octave Amount of Octaves in the pyramid (An octave can have muliples subsections/intervals)
        /// @param[in] interval Number of intervals in each octave/Nuber of levels per octave in the pyramid
        /// @param[in] starting_resolution Starting resolution for the point cloud. So that we can pre sample the point cloud,
        /// in order to reduce computation and define a standard.
        /// @param[in] starting_kp_grid_reso Resolution keypoint
        /// @note TODO Need to take a container of parameters instead of the descr_rad to generalize to different descriptors.
        GSHOTPyramid(pcl::PointCloud<PointType>::Ptr input,  int octaves, int interval,
                     float starting_resolution, float starting_kp_grid_reso, float descr_rad);
        
        // Destructor
        ~GSHOTPyramid();

        /** GETTERS AND SETTER **/
        
        /// Returns whether the pyramid is empty. An empty pyramid has no level.
        bool empty() const;
        
        /// Returns the number of levels per octave in the pyramid.
        int interval() const;
        
        /// Returns the pyramid levels.
        /// @note Scales are given by the following formula: 2^(1 - @c index / @c interval).
        const std::vector<Level> & levels() const;
//        //Get keypoints at a given level
//        Eigen::Vector3i getLayerTopology(int i);
//        pcl::PointCloud<DescriptorType> get_descriptors_layer(unsigned int);
//        pcl::PointCloud<PointType> get_keypoints_layer(unsigned int);
//        int get_octaves();
//        int get_sub_between_octaves();
//        int get_height();
//        float get_original_resolution();
//        float get_layer_resolution (int i);
        
//        //Type accessors
//        static const char* get_descriptor_type();
//        static const char* get_point_type();
        
//        // Pyramid informations
//        void toString();
        
        /** OTHERS **/
    
        
        /// Returns the convolutions of the pyramid with a filter.
        /// @param[in] filter Filter.
        /// @param[out] convolutions Convolution of each level.
        void convolve(const Level & filter, std::vector<Tensor> & convolutions) const;
        
        /// Returns the flipped version (horizontally) of a level.
        static GSHOTPyramid::Level Flip(const GSHOTPyramid::Level & level);
        
        /// Maps a pyramid level to a simple matrix (useful to apply standard matrix operations to it).
        /// @note The size of the matrix will be rows x (cols * NbFeatures).
        static Eigen::Map<Matrix, Eigen::Aligned> Map(Level & level);
        
        /// Maps a const pyramid level to a simple const matrix (useful to apply standard matrix
        /// operations to it).
        /// @note The size of the matrix will be rows x (cols * NbFeatures).
        static const Eigen::Map<const Matrix, Eigen::Aligned> Map(const Level & level);
        
        
        
        private:
        
        // Method for computing feature spaces. Will have different implementations depending on the descriptor used.
        PointCloudPtr
        compute_descriptor(PointCloudPtr input, PointCloudPtr keypoints, float);
        
        // Method creating the keypoint grid using the min/max values of the input
        PointCloudPtr
        compute_keypoints(PointCloudPtr input, float grid_reso, PointType min, PointType max);
        
        std::vector<typename pcl::PointCloud<DescriptorType>::Ptr >* _descriptors;
        // The corresponding positions of the descriptors in the space for each level
        std::vector<typename pcl::PointCloud<PointType>::Ptr >* _keypoints;
        
        
//        // Container of the different descriptor layers from 0 (original resolution) to n (lowest resolution, last octave)
//        std::vector<typename pcl::PointCloud<DescriptorType>::Ptr >* _descriptors;
        

        

        // Computes the 2D convolution of a pyramid level with a filter
        static void Convolve(const Level & x, const Level & y, Matrix & z);
        
//        // Number of keypoints per dimension (needed for the sliding box process)
//        std::vector<Eigen::Vector3i,Eigen::aligned_allocator<Eigen::Vector3i> > topology;
//        int _octaves;
//        float _original_resolution;
//        int interval_;
//        int height_;
//        Tensor<Cell> levels_;

        Eigen::Vector3i pad_;
        int interval_;
        // Represent a vector of 3D scene of descriptors computed at different resolution
        //from 0 (original resolution) to n (lowest resolution, last octave)
        std::vector<Level> levels_;
        //TODO: I don't think we need it
        // The corresponding positions of the descriptors in the space for each level
        std::vector<PointCloudPtr > _keypoints;
    };
    
    /// Serializes a pyramid to a stream.
    std::ostream & operator<<(std::ostream & os, const GSHOTPyramid & pyramid);
    
    /// Unserializes a pyramid from a stream.
    std::istream & operator>>(std::istream & is, GSHOTPyramid & pyramid);
}

#endif

