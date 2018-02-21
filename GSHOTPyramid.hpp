#ifndef FFLD_GSHOTPYRAMID_H
#define FFLD_GSHOTPYRAMID_H
//TODO CHANGE PADX PADY PADZ to (

#include "JPEGImage.h"

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
#include "typedefs.hpp"

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
            
            Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> row( int i) const{
                Eigen::Matrix<Type, this->dimension(0), this->dimension(1), Eigen::RowMajor> res;
                return res.setZero();
            }
            
        };
        
        /// Type of a pyramid level cell (fixed-size array of length NbFeatures).
        typedef Eigen::Array<Scalar, NbFeatures, 1> Cell;
        
        /// Type of a pyramid level (matrix of cells).
        typedef Tensor<Cell> Level;
        
        
        void test() const;
        /// Constructs an empty pyramid. An empty pyramid has no level.
        GSHOTPyramid();
        
        // Copy constructor
        GSHOTPyramid(const GSHOTPyramid&);
        
        /*
         * Main constructor, using the point cloud input, the descriptor applied, the number of octaves and the number of subsections by octave.
         * We use a starting resolution to pre sample the point cloud, in order to reduce computation and define a standard.
         * Need to take a container of parameters instead of the descr_rad to generalize to different descriptors.
         */
        GSHOTPyramid(typename pcl::PointCloud<PointType>::Ptr input,  int octaves, int interval, float starting_resolution, float starting_kp_grid_reso, float descr_rad);
        
        // Destructor
        ~GSHOTPyramid();
        
        /// Constructs a pyramid from the JPEGImage of a Scene.
        /// @param[in] image The JPEGImage of the Scene.
        /// @param[in] padx Amount of horizontal zero padding (in cells).
        /// @param[in] pady Amount of vertical zero padding (in cells).
        /// @param[in] interval Number of levels per octave in the pyramid.
        /// @note The amount of padding and the interval should be at least 1.
        GSHOTPyramid(const JPEGImage & image, int padx, int pady, int interval = 5);
        
        /// Constructs a pyramid from parameters and a list of levels.
        /// @param[in] padx Amount of horizontal zero padding (in cells).
        /// @param[in] pady Amount of vertical zero padding (in cells).
        /// @param[in] interval Number of levels per octave in the pyramid.
        /// @param[in] levels List of pyramid levels.
        /// @note The amount of padding and the interval must both be at least 1.
        /// @note The input levels are swapped with empty ones on exit.
        GSHOTPyramid(int padx, int pady, int interval, std::vector<Level> & levels);
        
        /// Returns whether the pyramid is empty. An empty pyramid has no level.
        bool empty() const;
        
        /// Returns the amount of horizontal zero padding (in cells).
        int padx() const;
        
        /// Returns the amount of vertical zero padding (in cells).
        int pady() const;
        
        /// Returns the number of levels per octave in the pyramid.
        int interval() const;
        
        /// Returns the pyramid levels.
        /// @note Scales are given by the following formula: 2^(1 - @c index / @c interval).
        const std::vector<Level> & levels() const;
        
        /// Returns the convolutions of the pyramid with a filter.
        /// @param[in] filter Filter.
        /// @param[out] convolutions Convolution of each level.
        void convolve(const Level & filter, std::vector<Matrix> & convolutions) const;
        
        /// Returns the flipped version (horizontally) of a level.
        static GSHOTPyramid::Level Flip(const GSHOTPyramid::Level & level);
        
        /// Maps a pyramid level to a simple matrix (useful to apply standard matrix operations to it).
        /// @note The size of the matrix will be rows x (cols * NbFeatures).
        static Eigen::Map<Matrix, Eigen::Aligned> Map(Level & level);
        
        /// Maps a const pyramid level to a simple const matrix (useful to apply standard matrix
        /// operations to it).
        /// @note The size of the matrix will be rows x (cols * NbFeatures).
        static const Eigen::Map<const Matrix, Eigen::Aligned> Map(const Level & level);
        
        // Getters
        pcl::PointCloud<DescriptorType> get_descriptors_layer(unsigned int);
        pcl::PointCloud<PointType> get_keypoints_layer(unsigned int);
        int get_octaves();
        int get_sub_between_octaves();
        int get_height();
        float get_original_resolution();
        float get_layer_resolution (int i);
        
        //Type accessors
        static const char* get_descriptor_type();
        static const char* get_point_type();
        
        // Pyramid informations
        void toString();
        
        private:
        
        // Method for computing feature spaces. Will have different implementations depending on the descriptor used.
        typename pcl::PointCloud<DescriptorType>::Ptr compute_space(typename pcl::PointCloud<PointType>::Ptr input, typename pcl::PointCloud<PointType>::Ptr keypoints, float);
        
        // Method creating the keypoint grid using the min/max values of the input
        typename pcl::PointCloud<PointType>::Ptr compute_keypoints(typename pcl::PointCloud<PointType>::Ptr input, float grid_reso, PointType min, PointType max);
        
        // Container of the different descriptor layers from 0 (original resolution) to n (lowest resolution, last octave)
        std::vector<typename pcl::PointCloud<DescriptorType>::Ptr >* _descriptors;
        
        // Container of the different keypoint layers from 0 (original resolution) to n (lowest resolution, last octave)
        std::vector<typename pcl::PointCloud<PointType>::Ptr >* _keypoints;
        
        int _octaves;
        float _original_resolution;
        // Efficiently computes Histogram of Oriented Gradient (HOG) features
        // Code to compute HOG features as described in "Object Detection with Discriminatively Trained
        // Part Based Models" by Felzenszwalb, Girshick, McAllester and Ramanan, PAMI 2010
        static void Hog(const JPEGImage & image, Level & level, int padx = 1, int pady = 1,
                        int cellSize = 8);
        
        // Computes the 2D convolution of a pyramid level with a filter
        static void Convolve(const Level & x, const Level & y, Matrix & z);
        

        
        int padx_;
        int pady_;
        int interval_;
        int height_;
        std::vector<Level> levels_;
    };
    
    /// Serializes a pyramid to a stream.
    std::ostream & operator<<(std::ostream & os, const GSHOTPyramid & pyramid);
    
    /// Unserializes a pyramid from a stream.
    std::istream & operator>>(std::istream & is, GSHOTPyramid & pyramid);
}

#endif

