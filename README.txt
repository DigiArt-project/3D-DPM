
This work has been done during the European project: "DigiArt".

                                  INTRODUCTION

This code implement a linear Structural Latent SVM for 3D point clouds objects
detection.
The relevant parts positions are used as latent variable for the SVM. We use
the SHOT descriptor to describe parts of the point cloud. In order to deal
with non-uniform sparsity of the point clouds, the scene is uniformly
discretized at a specified resolution (in GSHOTPyramid). Unfortunately, because
of time constraints, the code is not finished and there are features in the
code which may need to be rewritten.
Currently, adding parts for the detection doesn't improve the detection score
because the rotation estimations of the bounding boxes are not accurate enought.
Therefore, the algorithm look for the parts at wrong places and make the score
drops. In resume, the current program is not robust to rotation.


                                  How it works

Training:
        For all the "Scene" given, the "Mixture" identify and extract the
        positives and negatives "Model" samples. Then, it feed them to the
        Structural Latent SVM which use the LBFGS algorithm to create the
        "Model" that discriminate the best the "Objects" you want to find from
        the others.
Test:
        Compute the SVM score of the "Model" filter and all the corresponding
        parts of the scene (see Convolve() function in GSHOTPyramid). The
        scores are then compared to a threshold, and the bounding box of the
        highest scores are then shown to the user on the point cloud scene.


The models are stored in a text file (tmp.txt by default) format with the
following grammar:

Mixture := nbModels Model*
Model := nbParts bias Part*
Part := nbRows nbCols nbDepths nbFeatures xOffset yOffset zOffset deformations
filter_values*

nbModels is the number of mixture components (models). In our case, it will
always be equal to 1 because the goal was to get rid of the several components
by making the algorithm robust to rotation.
nbParts is the number of parts (including the root) in the model.
The bias is the offset to add to the scores of the model.
nbRows, nbCols, nbDepths, nbFeatures are the dimensions of the part filter.
xOffset, yOffset, zOffset are the offsets of the part relative to the root
(anchor). Deformations represents the deformation coefficients.
filter_values are the filter coefficients, stored
in row-major order, and of size nbRows x nbCols x nbDepths x nbFeatures.

The code is structured as follows:

    - main.cpp : File used to train and test the DPM 3D.
    - createscene.cpp : File used to artificially create scenes from different point
      clouds.
    - typedefs.h : All typedef definitions.
    - viewer.h : Wrapper around PCLVisualizer.

    - GSHOTPyramid: Constructs a pyramid from the PointCloud of a Scene.
    - Model: The Model class can represent both a deformable part-based model or
      a training sample with fixed latent variables (parts' positions).
    - Mixture: The Mixture class is made for assembling all the other classes in
      order to train and use the models used for detection.
    - LBFGS: Limited-memory BFGS (L-BFGS) algorithm implementation as described by
      Nocedal.
    - Scene: The Scene class consist of a (filename to a) pointcloud file and the
      list of objects present in it.
    - Object: The Object class represents an object in a Scene. It stores all the
      information regarding the object like its bounding box.
    - Rectangle: The Rectangle class defines a bounding box.
    - Tensor3D: The Tensor3D class defines a set of functions used by 3D tensors.
    - Intersector: Functor used to test for the intersection of two Rectangles.



Author : Fisichella Thomas
Date : 28/11/2018
