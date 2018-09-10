#ifndef DEF_POSITIONMANAGER_POSITIONMANAGERBASE_H
#define DEF_POSITIONMANAGER_POSITIONMANAGERBASE_H

#include <iostream>
#include <sys/time.h>
#include <string>

#include <infuse_envire/graph/EnvireGraph.hpp>

namespace PositionManager
{

typedef envire::core::FrameId FrameId; //Is a std::string
typedef envire::core::Transform Transform;
typedef std::string PoseId;
typedef base::Matrix6d Covariance;
typedef long long TimeUs;

class TimeManager
{
    public:

    static const unsigned int USEC_PER_SEC = 1000000;

    static TimeUs now()
    {
        timeval tv;
        gettimeofday(&tv,NULL);

        return USEC_PER_SEC*tv.tv_sec + tv.tv_usec;
    }

    static std::string toString(TimeUs time)
    {
        char str[512];
        int i;
        timeval tv;
        struct tm timeinfo;
        
        tv.tv_sec = time / USEC_PER_SEC;
        tv.tv_usec = time % USEC_PER_SEC;
        
        localtime_r(&tv.tv_sec, &timeinfo);
        i = strftime(str, 512, "%F %T.", &timeinfo);
        sprintf(str + i, "%6ld", tv.tv_usec);

        return std::string(str);
    }
    
    static std::string toStringShort(TimeUs time)
    {
        char str[512];
        int i;
        timeval tv;
        struct tm timeinfo;
        
        tv.tv_sec = time / USEC_PER_SEC;
        tv.tv_usec = time % USEC_PER_SEC;
        
        localtime_r(&tv.tv_sec, &timeinfo);
        i = strftime(str, 512, "%T.", &timeinfo);
        sprintf(str + i, "%6ld", tv.tv_usec);

        return std::string(str);
    }
};

struct FrameIdPair
{
    FrameIdPair(PositionManager::FrameId p = PositionManager::FrameId("parent"),
                PositionManager::FrameId c = PositionManager::FrameId("child")) :
        parent(p), child(c) {}

    PositionManager::FrameId parent;
    PositionManager::FrameId child;
};

PositionManager::PoseId getPoseId(const FrameId& parent, const FrameId& child)
{
    return parent + "To" + child;
}


class Pose 
{
    public :

    Pose() {}
    Pose(const FrameId& parent, const FrameId& child, const Transform& tr) : 
        _parent(parent), _parentTime(-1), _child(child), _childTime(-1), _tr(tr) {}
    Pose(const Pose& pose) : 
        _parent(pose._parent), _parentTime(pose._parentTime),  _child(pose._child), _childTime(pose._childTime), _tr(pose._tr) {}

    std::string toString() const
    { 
        return _parent + "->" + _child + "\n" + this->toStringShort(_tr);
    }

    std::string toStringVerboseShort() const
    {
        std::ostringstream stream;
        stream << _parent << " " << PositionManager::TimeManager::toStringShort(_parentTime) 
               << "\n->\n"
               << _child  << " " << PositionManager::TimeManager::toStringShort(_parentTime)
               << "\n" << this->toStringShort(_tr);
        return stream.str();
    }

    std::string toStringVerbose() const
    {
        std::ostringstream stream;
        stream << _parent <<" "<< PositionManager::TimeManager::toString(_parentTime)
               << "\n->\n"
               << _child  <<" "<< PositionManager::TimeManager::toString(_parentTime)
               << "\n" << this->toString(_tr) << "\n" << _tr.transform.cov;

        return stream.str();
    }

    std::string toString(const Transform& tr) const
    {
        std::ostringstream oss;
        oss << "t: (" << tr.transform.translation(0) << " " 
                      << tr.transform.translation(1) << " " 
                      << tr.transform.translation(2) << ")\n"
            << "r: (" << tr.transform.orientation.w() << " " 
                      << tr.transform.orientation.x() << " " 
                      << tr.transform.orientation.y() << " " 
                      << tr.transform.orientation.z() << ")"; 

        return oss.str();
    }
    
    std::string toStringShort(const Transform& tr) const
    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed, std::ios::floatfield);
        oss.precision(2);
        
        oss << "t: (" << tr.transform.translation(0) << " " 
                      << tr.transform.translation(1) << " " 
                      << tr.transform.translation(2) << ")\n"
            << "r: (" << tr.transform.orientation.w() << " " 
                      << tr.transform.orientation.x() << " " 
                      << tr.transform.orientation.y() << " " 
                      << tr.transform.orientation.z() << ")"; 

        return oss.str();
    }

    PoseId getPoseId() const
    {
        return PositionManager::getPoseId(_parent,_child);
    }

    public:

    FrameId _parent;
    TimeUs _parentTime;
    FrameId _child;
    TimeUs _childTime;
    Transform _tr;
};

class Graph : public envire::core::EnvireGraph
{
    // write accessors for vertices

    public:

    Graph();

    void copyFrom(Graph& src);
    // set the rootnode name to avoid root node reaturn in leaves if the root node has only one child. (In an undirected tree, the root node is undefined)
    std::list<FrameId> getLeaves(const PositionManager::FrameId* rootNode = NULL);
};

};

#endif

