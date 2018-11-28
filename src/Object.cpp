#include "Object.h"

#include <iostream>

using namespace FFLD;
using namespace std;

Object::Object() : name_(UNKNOWN), pose_(UNSPECIFIED), truncated_(false), difficult_(false)
{
}

Object::Object(Name name, Pose pose, bool truncated, bool difficult, Rectangle bndbox, Eigen::Vector3i rgb) :
name_(name), pose_(pose), truncated_(truncated), difficult_(difficult), bndbox_(bndbox), rgb_(rgb)
{
}

bool Object::empty() const
{
	return (name() == UNKNOWN) && (pose() == UNSPECIFIED) && !truncated() && !difficult() &&
           bndbox().empty();
}

Object::Name Object::name() const
{
	return name_;
}

void Object::setName(Name name)
{
	name_ = name;
}

Object::Pose Object::pose() const
{
	return pose_;
}

void Object::setPose(Pose pose)
{
	pose_ = pose;
}

bool Object::truncated() const
{
	return truncated_;
}

void Object::setTruncated(bool truncated)
{
	truncated_ = truncated;
}

bool Object::difficult() const
{
	return difficult_;
}

void Object::setDifficult(bool difficult)
{
	difficult_ = difficult;
}

Rectangle Object::bndbox() const
{
	return bndbox_;
}

void Object::setBndbox(Rectangle bndbox)
{
	bndbox_ = bndbox;
}

Eigen::Vector3i Object::color() const{
    return rgb_;
}

void Object::setColor(Eigen::Vector3i rgb){
    rgb_ = rgb;
}

ostream & FFLD::operator<<(ostream & os, const Object & obj)
{
	return os << static_cast<int>(obj.name()) << ' ' << static_cast<int>(obj.pose()) << ' '
			  << obj.truncated() << ' ' << obj.difficult() << ' ' << obj.bndbox();
}

istream & FFLD::operator>>(istream & is, Object & obj)
{
	int name, pose;
	bool truncated, difficult;
	Rectangle bndbox;
	
    is >> name >> pose >> truncated >> difficult >> bndbox;
	
	if (!is) {
		obj = Object();
		return is;
	}
	
	obj.setName(static_cast<Object::Name>(name));
	obj.setPose(static_cast<Object::Pose>(pose));
	obj.setTruncated(truncated);
	obj.setDifficult(difficult);
	obj.setBndbox(bndbox);
	
    return is;
}
