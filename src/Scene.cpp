#include "Scene.h"

#include <algorithm>
#include <iostream>
#include <sstream>

#include <libxml/parser.h>

using namespace FFLD;
using namespace std;

Scene::Scene()/* : width_(0), height_(0), depth_(0)*/
{
}

Scene::Scene(/*Eigen::Vector3i origin, /*int depth, int height, int width, */const string & filename,
             const vector<Object> & objects) : /*origin_(origin), /*width_(width), height_(height), depth_(depth),*/
pcFileName_(filename), objects_(objects)
{
}

bool Scene::empty() const
{
    return (/*(width() <= 0) || (height() <= 0) || (depth() <= 0) || */filename().empty()) &&
		   objects().empty();
}

template <typename Result>
static inline Result content(const xmlNodePtr cur)
{
	if ((cur == NULL) || (cur->xmlChildrenNode == NULL))
		return Result();
	
	istringstream iss(reinterpret_cast<const char *>(cur->xmlChildrenNode->content));
	Result result;
	iss >> result;
	return result;
}

Scene::Scene(const string & xmlName, const string & pcFileName, const float resolution)
    : pcFileName_(pcFileName), resolution_(resolution)
{

//    PointCloudPtr cloud( new PointCloudT);

//    if (readPointCloud(pcFileName_, cloud) == -1) {
//        cout<<"couldnt open point cloud file"<<endl;
//    }

//    PointType min;
//    PointType max;
//    pcl::getMinMax3D(*cloud, min, max);

//    origin_ = Eigen::Vector3i(floor(min.z/resolution),
//                              floor(min.y/resolution),
//                              floor(min.x/resolution));
	
    xmlDoc * doc = xmlParseFile(xmlName.c_str());
	
    if (doc == NULL) {
        cerr << "Could not open " << xmlName << endl;
        return;
    }
	
    xmlNodePtr cur = xmlDocGetRootElement(doc);
	
    if (cur == NULL) {
        xmlFreeDoc(doc);
        cerr << "Could not open " << xmlName << endl;
        return;
    }
	
    if (xmlStrcmp(cur->name, reinterpret_cast<const xmlChar *>("annotation"))) {
        xmlFreeDoc(doc);
        cerr << "Could not open " << xmlName << endl;
        return;
    }
	
    cur = cur->xmlChildrenNode;
	
    while (cur != NULL) {
        if (!xmlStrcmp(cur->name, reinterpret_cast<const xmlChar *>("label"))) {

            xmlChar *className;
            xmlChar *obboxChar;
            xmlChar *aabboxChar;
            xmlChar *localPoseChar;
            xmlChar *colorChar;
            Object::Name objName;


            className = xmlGetProp(cur, reinterpret_cast<const xmlChar *>("text"));
            if( !xmlStrcmp(className, reinterpret_cast<const xmlChar *>("chair"))){
//                cout<<"Scene:: found a chair"<<endl;
                objName = Object::CHAIR;
            } else{
                objName = Object::AEROPLANE;
            }
            obboxChar = xmlGetProp(cur, reinterpret_cast<const xmlChar *>("obbox"));
            aabboxChar = xmlGetProp(cur, reinterpret_cast<const xmlChar *>("aabbox"));
            localPoseChar = xmlGetProp(cur, reinterpret_cast<const xmlChar *>("local_pose"));
            colorChar = xmlGetProp(cur, reinterpret_cast<const xmlChar *>("color"));

            vector<float> obbox;
            vector<float> aabbox;
            vector<float> localPose;
            vector<int> color;
            string obboxStr = (char *) obboxChar;
            string aabboxStr = (char *) aabboxChar;
            string localPoseStr = (char *) localPoseChar;
            string colorStr = (char *) colorChar;


            size_t first = 0, last = 0;
            while ( ( (last = obboxStr.find(" ", last)) != string::npos)){
                obbox.push_back( stof( obboxStr.substr( first, last - first)));
                ++last;
                first = last;
            }
            obbox.push_back( stof( obboxStr.substr( first, obboxStr.length() - first)));

            if( obbox.size() != 10){
                cerr<<"Xml oriented bounding box is not correct"<<endl;
                return;
            }

            Eigen::Vector3f origin(obbox[2], obbox[1], obbox[0]);
            Eigen::Vector3f sizes(obbox[5], obbox[4], obbox[3]);

            Eigen::Matrix4f orientationTransform = Eigen::Matrix4f::Identity();
            orientationTransform.topLeftCorner(3, 3) = Eigen::Quaternionf( obbox[9], obbox[6], obbox[7], obbox[8]).
                    toRotationMatrix();
            Rectangle obndbox( origin, sizes, orientationTransform);

            first = 0, last = 0;
            while ( ( (last = aabboxStr.find(" ", first)) != string::npos)){
                aabbox.push_back( stof( aabboxStr.substr( first, last - first)));
                ++last;
                first = last;
            }
            aabbox.push_back( stof( aabboxStr.substr( first, aabboxStr.length() - first)));

            if( aabbox.size() != 6){
                cerr<<"Xml aa bounding box is not correct : "<< aabbox.size() <<endl;
                return;
            }

            first = 0, last = 0;
            while ( ( (last = colorStr.find(" ", first)) != string::npos)){
                color.push_back( stof( colorStr.substr( first, last - first)));
                ++last;
                first = last;
            }
            color.push_back( stof( colorStr.substr( first, colorStr.length() - first)));

            if( color.size() != 3){
                cerr<<"Xml color is not correct : "<< color.size() <<endl;
                return;
            }

            Eigen::Vector3i rgb(color[0], color[1], color[2]);

            Eigen::Vector3f minPt(aabbox[2], aabbox[1], aabbox[0]);
            Eigen::Vector3f maxPt(aabbox[5], aabbox[4], aabbox[3]);
            Eigen::Vector3f aaBoxSizes(maxPt(0)-minPt(0), maxPt(1)-minPt(1), maxPt(2)-minPt(2));

            // absolute bndbox positions
            Rectangle aabndbox( minPt, aaBoxSizes);


//            if(objName == Object::CHAIR) cout<<"Scene:: absolute chair bndbox : "<<aabndbox<<endl;

            Object obj( objName, Object::FRONTAL, false, false, aabndbox, rgb);
//            Object obj( objName, Object::FRONTAL, false, false, obndbox);


            objects_.push_back( obj);

            first = 0, last = 0;
            while ( ( (last = localPoseStr.find(" ", first)) != string::npos)){
                localPose.push_back( stof( localPoseStr.substr( first, last - first)));
                ++last;
                first = last;
            }
            localPose.push_back( stof( localPoseStr.substr( first, localPoseStr.length() - first)));

            if( localPose.size() != 4){
                cerr<<"Xml local pose is not correct : "<< localPose.size() <<endl;
                return;
            }
            localPose_.push_back( Eigen::Vector3f(localPose[0], localPose[1], localPose[2]/*, localPose[3]*/));
        }
        cur = cur->next;
    }
	
    xmlFreeDoc(doc);
//    cout<<"Scene:: objects().size() : "<<objects().size()<<endl;
}



const string & Scene::filename() const
{
    return pcFileName_;
}

void Scene::setFilename(const string &filename)
{
    pcFileName_ = filename;
}

const vector<Object> & Scene::objects() const
{
	return objects_;
}

void Scene::setObjects(const vector<Object> &objects)
{
	objects_ = objects;
}

const std::vector<Eigen::Vector3f> & Scene::localPose() const{
    return localPose_;
}

float Scene::resolution() const{
    return resolution_;
}

ostream & FFLD::operator<<(ostream & os, const Scene & scene)
{
    os /*<< scene.origin()(0) << ' ' << scene.origin()(1) << ' ' << scene.origin()(2) << ' '
       /*<< scene.depth() << ' ' << scene.height() << ' ' << scene.width() << ' '*/
	   << scene.objects().size() << ' ' << scene.filename() << endl;
	
	for (int i = 0; i < scene.objects().size(); ++i)
		os << scene.objects()[i] << endl;
	
	return os;
}

istream & FFLD::operator>>(istream & is, Scene & scene)
{
    int /*x, y, z, /*width, height, depth,*/ nbObjects;
    
    is /*>> z >> y >> x /*>> depth >> height >> width*/ >> nbObjects;
	is.get(); // Remove the space
	
	string filename;
	getline(is, filename);
	
	vector<Object> objects(nbObjects);
	
	for (int i = 0; i < nbObjects; ++i)
		is >> objects[i];
	
	if (!is) {
		scene = Scene();
		return is;
	}
	
    scene = Scene(/*Eigen::Vector3i(z, y, x), /*depth, height, width, */filename, objects);
	
	return is;
}
