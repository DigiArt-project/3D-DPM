//--------------------------------------------------------------------------------------------------
// Written by Fisichella Thomas
// Date 25/05/2018
//
// This file is part of FFLDv2 (the Fast Fourier Linear Detector version 2)
//
// FFLDv2 is free software: you can redistribute it and/or modify it under the terms of the GNU
// Affero General Public License version 3 as published by the Free Software Foundation.
//
// FFLDv2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero
// General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License along with FFLDv2. If
// not, see <http://www.gnu.org/licenses/>.
//--------------------------------------------------------------------------------------------------

#include "Scene.h"

#include <algorithm>
#include <iostream>
#include <sstream>

#include <libxml/parser.h>

using namespace FFLD;
using namespace std;

Scene::Scene() : width_(0), height_(0), depth_(0)
{
}

Scene::Scene(Eigen::Vector3i origin, int depth, int height, int width, const string & filename,
             const vector<Object> & objects) : origin_(origin), width_(width), height_(height), depth_(depth),
filename_(filename), objects_(objects)
{
}

bool Scene::empty() const
{
	return ((width() <= 0) || (height() <= 0) || (depth() <= 0) || filename().empty()) &&
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

Scene::Scene(const string & filename, const float resolution)
{
    const string Names[20] =
    {
        "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
        "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa",
        "train", "tvmonitor"
    };
	
    const string Poses[4] =
    {
        "Frontal", "Left", "Rear", "Right"
    };
	
    xmlDoc * doc = xmlParseFile(filename.c_str());
	
    if (doc == NULL) {
        cerr << "Could not open " << filename << endl;
        return;
    }
	
    xmlNodePtr cur = xmlDocGetRootElement(doc);
	
    if (cur == NULL) {
        xmlFreeDoc(doc);
        cerr << "Could not open " << filename << endl;
        return;
    }
	
    if (xmlStrcmp(cur->name, reinterpret_cast<const xmlChar *>("annotation"))) {
        xmlFreeDoc(doc);
        cerr << "Could not open " << filename << endl;
        return;
    }
	
    cur = cur->xmlChildrenNode;
	
    while (cur != NULL) {
//        if (!xmlStrcmp(cur->name, reinterpret_cast<const xmlChar *>("label"))) {

//            xmlChar *className;
//            xmlChar *obboxChar;

//            className = xmlGetProp(cur, "text");
//            obboxChar = xmlGetProp(cur, "obbox");

//            vector<float> obbox;


//            string obboxStr = (char *) obboxChar;
//            size_t first = 0, last = 0;
//            while ( ( (last = obboxStr.find(" ", last)) != npos)){
//                obbox.push_back( stof( obboxStr.substr( first, last - first)));
//                ++last;
//                first = last;
//            }
//            obbox.push_back( stof( obboxStr.substr( first, obboxStr.length() - first)));

//            if( obbox.size() != 10){
//                cerr<<"Xml oriented bounding box is not correct"<<endl;
//                return;
//            }
//            Eigen::Vector3f boxCenter(obbox(0), obbox(1), obbox(2));
//            Eigen::Vector3f boxSize(obbox(3), obbox(4), obbox(5));
//            Eigen::Matrix3f orientationTransform ( Eigen::Quaternion( obbox(9), obbox(6), obbox(7), obbox(8)));
//            Vector3i origin();
//            Rectangle bndbox( origin, depth, row, col, resolution);
//            Object obj( (char*)className, Object::FRONTAL, false, false, bndbox);

//        }
        cur = cur->next;
    }
	
    xmlFreeDoc(doc);
}




Eigen::Vector3i Scene::origin() const{
    return origin_;
}

void Scene::setOrigin(Eigen::Vector3i origin){
    origin_ = origin;
}


int Scene::width() const
{
	return width_;
}

void Scene::setWidth(int width)
{
	width_ = width;
}

int Scene::height() const
{
	return height_;
}

void Scene::setHeight(int height)
{
	height_ = height;
}

int Scene::depth() const
{
	return depth_;
}

void Scene::setDepth(int depth)
{
	depth_ = depth;
}

const string & Scene::filename() const
{
	return filename_;
}

void Scene::setFilename(const string &filename)
{
	filename_ = filename;
}

const vector<Object> & Scene::objects() const
{
	return objects_;
}

void Scene::setObjects(const vector<Object> &objects)
{
	objects_ = objects;
}

ostream & FFLD::operator<<(ostream & os, const Scene & scene)
{
    os << scene.origin()(0) << ' ' << scene.origin()(1) << ' ' << scene.origin()(2) << ' '
       << scene.depth() << ' ' << scene.height() << ' ' << scene.width() << ' '
	   << scene.objects().size() << ' ' << scene.filename() << endl;
	
	for (int i = 0; i < scene.objects().size(); ++i)
		os << scene.objects()[i] << endl;
	
	return os;
}

istream & FFLD::operator>>(istream & is, Scene & scene)
{
    int x, y, z, width, height, depth, nbObjects;
    
    is >> z >> y >> x >> depth >> height >> width >> nbObjects;
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
	
    scene = Scene(Eigen::Vector3i(z, y, x), depth, height, width, filename, objects);
	
	return is;
}
