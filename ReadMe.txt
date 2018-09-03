Author : Fisichella Thomas
Date : 25/05/2018



This work has been done during the European project : DigiArt.

DPM 3D :

How it works :

In order to deal with non-uniform sparsity of 3D point clouds, the scene is uniformly discretized at a specified resolution (in GSHOTPyramid).
Training :
	For all the Scene given, the Mixture identify and extract the positives and negatives Model samples.
	Then, it feed them to the Structural Latent SVM which use the LBFGS algorithm to create the Model that 
	discriminate the best the Objects you want to find from the others.
	You can use ~/3DDataset/3DDPM/chair_normalized/  or other dataset stored in /media/ubuntu/DATA/3DDataset/ for training

Test :
	Compute the SVM score of the Model filter and all the corresponding parts of the scene (see Convolve function in GSHOTPyramid). 
	The scores are then compared to a threshold, and the bounding box of the highest scores are then shown to the user on the 
	point cloud scene.

The code is structured as follow :

- GSHOTPyramid : Constructs a pyramid from the PointCloud of a Scene.
- Model : The Model class can represent both a deformable part-based model or a training sample with
fixed latent variables (parts' positions).
- Mixture : The Mixture class is made for assembling all the other classes in order to train and use the models used for detection.
- LBFGS : Limited-memory BFGS (L-BFGS) algorithm implementation as described by Nocedal.
- Scene : The Scene class consist of a (filename to a) pointcloud file and the list
of objects present in it. It is only used for training.
- Object : The Object class represents an object in a Scene. It stores all the information regarding the object
like its bounding box.
- Rectangle : The Rectangle class defines a bounding box.
- Tensor3D : The Tensor3D class defines a set of functions used by 3D tensors.
- Intersector : Functor used to test for the intersection of two Rectangles.
- typedefs.h : All typedef definitions.
- viewer.h : Wrapper around PCLVisualizer.
- realTrain.cpp : File used to train and test the DPM 3D.
- test_<...>.cpp : All file used for testing the project.
- createscene.cpp : File used to artificially create scenes from different point clouds.



TODO :

- Normalize the SVM score using Platt's sclaing method.
- Fix the parts positions regarding the root (or the opposite). Sometimes, the parts are outside the root bounding box. 
- Allow more interval and more octave in the pyramid (only tested and designed for 1 interval and 2 octaves).
This imply that currently, the size of the objects during training and testing has to be approximately the same.
- Test the algorithm on real datasets (see SceneNN dataset)
- Allow the project to have more than one Model in the Mixture.
- Set the starting_resolution of the GSHOTPyramid constructor from the test file (for coherent results).
- Compute uneccessary scores during training.
- Let the user set the epsillon value used in Mixture.
- Check TODO comments in the code

- problem assigning score of dt3d to root offset => error in the detection score due to the parts
	Maybe t[x] should have an infinite value beyond the weight filter size ?
	Check distance Mahalanobis instead of L2 distance
- initPart ???
- use hard negative example
- use big dataset for tries
- Use EMD or main direction of the shot in root filter to make the filter invariant to the rotation (see picture)
- Doesnt work if object is not entirely in the scene
- maxNegSample has an impact on the detection score but looks normal



Infos :

An implicit connexion is made between the Rectangle bounding box size and the size of the Tensors used to carry the filter's
information (Rectangle size = Tensor3D size).
The real position of the origin of a Rectangle in a scene is found using this function :
	position <==> rectangle.origin() * rectangle.resolution()
You can compute the origin of a Rectangle from the position of a Tensor3D (z, y, x) using the following equation :
	rectangle.origin() <==> (z, y, x) - floor( scene->origin() * scale )    
	with "double scale = 1 / pow(2.0, static_cast<double>(lvl) / interval)"
	(You can find an example of this computation in the detect() function in realTrain.cpp)
	
Different metric distances are implemented (tensor3d.h : convolve(), khi2Convolve(), EMD()) to compute the score 
of the SVM equation decision. The program was designed for the linear approach (convolve()) so its the fastest one 
but using other metrics or kernels could be interesting.
