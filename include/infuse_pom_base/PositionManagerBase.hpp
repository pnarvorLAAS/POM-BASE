#ifndef DEF_POSITIONMANAGER_POSITIONMANAGERBASE_H
#define DEF_POSITIONMANAGER_POSITIONMANAGERBASE_H

#include <iostream>
#include <sys/time.h>
#include <string>

#include "config.h"

#ifdef USE_ENVIRE_MINIMAL
    #include <infuse_envire/graph/EnvireGraph.hpp>
#else
    #include <envire_core/graph/EnvireGraph.hpp>
#endif

namespace PositionManager
{

typedef envire::core::FrameId FrameId; //Is a std::string
typedef envire::core::Transform Transform;
typedef base::Matrix6d Covariance;
typedef envire::core::Path Path;
typedef long long TimeUs;

Transform identityTransform();
std::string toString(const Transform& tr);
std::string toStringShort(const Transform& tr);

class TimeManager
{
    public:

    static const unsigned int USEC_PER_SEC = 1000000;

    static TimeUs now();
    static std::string toString(TimeUs time);
    static std::string toStringShort(TimeUs time);
};

struct FrameIdPair
{
    FrameIdPair(PositionManager::FrameId p = PositionManager::FrameId("parent"),
                PositionManager::FrameId c = PositionManager::FrameId("child"));

    PositionManager::FrameId parent;
    PositionManager::FrameId child;
};

typedef std::string PoseId;
PositionManager::PoseId getPoseId(const FrameId& parent, const FrameId& child);

class Pose 
{
    public :

    Pose();
    Pose(const FrameId& parent, const FrameId& child, const Transform& tr = identityTransform());
    Pose(const Pose& pose);

    std::string toString() const;
    std::string toStringVerboseShort() const;
    std::string toStringVerbose() const;
    PoseId getPoseId() const;

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

