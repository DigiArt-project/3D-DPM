#ifndef FFLD_MODEL_H
#define FFLD_MODEL_H
#include <tuple>
#include "GSHOTPyramid.h"
#include <omp.h>




namespace FFLD
{
/// The Model class can represent both a deformable part-based model or a training sample with
/// fixed latent variables (parts' positions). In both cases the members are the same: a list of
/// parts and a bias. If it is a sample, for each part the filter is set to the corresponding
/// features, the offset is set to the part's position relative to the root, and the deformation is
/// set to the deformation gradient (<tt>dx^2 dx dy^2 dy dz^2 dz</tt>), where (<tt>dx dy dz<tt>) are
/// the differences between the reference part location and the part position. The dot product
/// between the deformation gradient and the model deformation then computes the deformation cost.
/// @note Define the PASCAL_MODEL_3D to also deform parts across scales.
class Model
{
public:
    /// Type of a 3d position (z y x lvl).
    typedef Eigen::Vector4i Position;
	
	/// Type of a matrix of 3d positions.
    typedef Tensor3D<Position> Positions;
	
	/// Type of a 3d quadratic deformation (dx^2 dx dy^2 dy dz^2 dz).
    typedef Eigen::Array<double, 8, 1> Deformation;
	
	/// The part structure stores all the information about a part (or the root).
	struct Part
	{
        GSHOTPyramid::Level filter;
        Position offset;			///< Part offset (dz dy dx) relative to the root.
		Deformation deformation;	///< Deformation cost (dx^2 dx dy^2 dy dz^2 dz).
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	
	/// Constructs an empty model. An empty model has an empty root and no part.
	Model();
	
	/// Constructs a model with the specified dimensions and initializes all the filters and the
	/// bias to zero.
	/// @param[in] rootSize Size of the root filter (<tt>rows x cols</tt>).
	/// @param[in] nbParts Number of parts (without the root).
	/// @param[in] partSize Size of all the parts (<tt>rows x cols</tt>).
	/// @note The model will be empty if any of the parameter is invalid.
    explicit Model(Eigen::Vector3i rootSize, int nbParts = 0,
                   Eigen::Vector3i partSize = Eigen::Vector3i(0, 0, 0));
	
	/// Constructs a model from a list of parts and a bias.
	explicit Model(const std::vector<Part> & parts, double bias = 0.0);
	
	/// Returns whether the model is empty. An empty model has an empty root and no part.
	bool empty() const;
	
	/// Returns the model parts (the first one is the root).
	const std::vector<Part> & parts() const;
	
	/// Returns the model parts (the first one is the root).
	std::vector<Part> & parts();
	
	/// Returns the model bias.
	double bias() const;
	
	/// Returns the model bias.
	double & bias();
	
	/// Returns the size of the root (<tt>rows x cols</tt>).
	/// @note Equivalent to std::pair<int, int>(parts()[0].rows(), parts()[0].cols()).
    Eigen::Vector3i rootSize() const;
	
	/// Returns the size of the parts (<tt>rows x cols</tt>).
	/// @note Equivalent to make_pair(parts()[1].rows(), parts()[1].cols()) if the model has parts.
    Eigen::Vector3i partSize() const;
	
	/// Initializes the specidied number of parts from the root.
	/// @param[in] nbParts Number of parts (without the root).
	/// @param[in] partSize Size of each part (<tt>rows x cols</tt>).
	/// @note The model stay unmodified if any of the parameter is invalid.
	/// @note The parts are always initialized at twice the root resolution.
    void initializeParts(int nbParts, Eigen::Vector3i partSize, GSHOTPyramid::Level root2x);
	
	/// Initializes a training sample with fixed latent variables from a specified position in
	/// a pyramid of features.
	/// @param[in] pyramid Pyramid of features.
	/// @param[in] x, y, z Coordinates of the root.
	/// @param[out] sample Initialized training sample.
	/// @param[in] positions Positions of each part for each pyramid level
	/// (<tt>parts x levels</tt>, only required if the model has parts).
	/// @note The sample will be empty if any of the parameter is invalid or if any of the part
	/// filter is unreachable.
    void initializeSample(const GSHOTPyramid & pyramid, int box, int z, int y, int x, int lvl, Model & sample,
                          const vector<vector<vector<Positions> > >* positions = 0) const;
	
    //Detection, energy computation
	/// Returns the scores of the convolutions + distance transforms of the parts with a pyramid of
	/// features.
	/// @param[in] pyramid Pyramid of features.
	/// @param[out] scores Scores for each pyramid level.
	/// @param[out] positions Positions of each part and each pyramid level.
	/// @param[in] Precomputed convolutions of each part and each pyramid level.
    void convolve(const GSHOTPyramid & pyramid, vector<vector<Tensor3DF> > & scores,
                  vector<vector<vector<Positions> > > *positions = 0,
                  vector<vector<vector<Tensor3DF> > > *convolutions = 0) const;
	
    //Similarity in the optimization process of the SVM
	/// Returns the dot product between the model and a fixed training @p sample.
	/// @note Returns NaN if the sample and the model are not compatible.
	/// @note Do not compute dot products between two models or between two samples.
	double dot(const Model & sample) const;
	
	/// Returns the norm of a model or a fixed training sample.
	double norm() const;
	
	/// Adds the filters, deformation costs, and bias of the fixed sample with the ones of
	/// @p sample.
	/// @note Do nothing if the models are incompatible.
	/// @note Do not use with models, only with fixed samples.
	Model & operator+=(const Model & sample);
	
	/// Subtracts the filters, deformation costs, and bias of the fixed sample with the ones of
	/// @p sample.
	/// @note Do nothing if the models are incompatible.
	/// @note Do not use with models, only with fixed samples.
	Model & operator-=(const Model & sample);
	
	/// Multiplies the filters, deformation costs, and bias of the fixed sample by @p a.
	/// @note Do not use with models, only with fixed samples.
    Model & operator*=(double a);
	
    /// Computes an in-place 3D quadratic distance transform.
	/// @param[in,out] matrix Matrix to tranform in-place.
	/// @param[in] part Part from which to read the deformation cost and offset.
    /// @param tmp Temporary tensors.
	/// @param[out] positions Optimal position of each part for each root location.
    static void DT3D(Tensor3DF & tensor, const Part & part, Tensor3DF & tmp1, Tensor3DF & tmp2,
                     Positions * positions = 0);

    Eigen::Vector3i boxSize_;
private:
    float* orientation_;//9 values (3x3 axis)
	std::vector<Part> parts_;
	double bias_;
};

/// Serializes a model to a stream.
std::ostream & operator<<(std::ostream & os, const Model & model);

/// Unserializes a model from a stream.
std::istream & operator>>(std::istream & is, Model & model);
}

#endif
