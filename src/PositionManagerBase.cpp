#include <infuse_pom_base/PositionManagerBase.hpp>

using namespace PositionManager;

Transform PositionManager::identityTransform()
{
    Transform res;
    res.setIdentity();
    res.time.microseconds = 0;

    return res;
}

std::string PositionManager::toString(const Transform& tr)
{
    std::ostringstream oss;
    oss << "time " << tr.time.microseconds << "\n"
        << "t: (" << tr.transform.translation(0) << " " 
                  << tr.transform.translation(1) << " " 
                  << tr.transform.translation(2) << ")\n"
        << "r: (" << tr.transform.orientation.w() << " " 
                  << tr.transform.orientation.x() << " " 
                  << tr.transform.orientation.y() << " " 
                  << tr.transform.orientation.z() << ")"; 

    return oss.str();
}

std::string PositionManager::toStringShort(const Transform& tr)
{
    std::ostringstream oss;
    oss.setf(std::ios::fixed, std::ios::floatfield);
    oss.precision(10);
    
    oss << "t: (" << tr.transform.translation(0) << " " 
                  << tr.transform.translation(1) << " " 
                  << tr.transform.translation(2) << ")\n"
        << "r: (" << tr.transform.orientation.w() << " " 
                  << tr.transform.orientation.x() << " " 
                  << tr.transform.orientation.y() << " " 
                  << tr.transform.orientation.z() << ")"; 

    return oss.str();
}

/////////////////////////////////////////////////
TimeUs TimeManager::now()
{
    timeval tv;
    gettimeofday(&tv,NULL);

    return USEC_PER_SEC*tv.tv_sec + tv.tv_usec;
}

std::string TimeManager::toString(TimeUs time)
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

std::string TimeManager::toStringShort(TimeUs time)
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


/////////////////////////////////////////////////
FrameIdPair::FrameIdPair(FrameId p, FrameId c) :
    parent(p), child(c)
{}

PoseId PositionManager::getPoseId(const FrameId& parent, const FrameId& child)
{
    return parent + "To" + child;
}

/////////////////////////////////////////////////
Pose::Pose()
{}

Pose::Pose(const FrameId& parent, const FrameId& child, const Transform& tr) : 
    _parent(parent),
    _parentTime(-1),
    _child(child),
    _childTime(-1),
    _tr(tr)
{}

Pose::Pose(const Pose& pose) : 
    _parent(pose._parent),
    _parentTime(pose._parentTime),
    _child(pose._child),
    _childTime(pose._childTime),
    _tr(pose._tr)
{}

std::string Pose::toString() const
{ 
    return _parent + "->" + _child + "\n" + toStringShort(_tr);
}

std::string Pose::toStringVerboseShort() const
{
    std::ostringstream stream;
    stream << _parent << " " << PositionManager::TimeManager::toStringShort(_parentTime) 
           << "\n->\n"
           << _child  << " " << PositionManager::TimeManager::toStringShort(_parentTime)
           << "\n" << toStringShort(_tr);
    return stream.str();
}

std::string Pose::toStringVerbose() const
{
    std::ostringstream stream;
    stream << _parent <<" "<< PositionManager::TimeManager::toString(_parentTime)
           << "\n->\n"
           << _child  <<" "<< PositionManager::TimeManager::toString(_parentTime)
           << "\n" << PositionManager::toString(_tr) << "\n" << _tr.transform.cov;

    return stream.str();
}

PoseId Pose::getPoseId() const
{
    return PositionManager::getPoseId(_parent,_child);
}

/////////////////////////////////////////////////
Graph::Graph()
{
}

void Graph::copyFrom(Graph& src)
{
    this->graph().clear();
    boost::copy_graph(src, this->graph());
    this->regenerateLabelMap();
}

std::list<FrameId> Graph::getLeaves(const PositionManager::FrameId* rootNode)
{
    // find nodes with no child
    // warning : there are always edges in both directions even for a directed graph. A leaf is therefore a node with only one child (its parent)

    std::list<FrameId> leafList;
    std::map<FrameId, unsigned int> nodeMap; //frame id + number of children

    typename boost::graph_traits<EnvireGraph>::vertex_iterator it, end;
    for(boost::tie(it,end) = boost::vertices(this->graph()); it != end; ++it)
    {
        nodeMap.insert(std::pair<FrameId,unsigned int>(getFrameId(*it),0));
    }

    // counting number of children for each node
    std::pair<EnvireGraph::edge_iterator, EnvireGraph::edge_iterator> edges_it = this->getEdges();
    for(EnvireGraph::edge_iterator it = edges_it.first; it != edges_it.second; it++)
    {
        FrameId sourceNode;
        sourceNode = getFrameId(source(*it, this->graph()));
        
        //std::cout << sourceNode << "->" << getFrameId(target(*it, this->graph())) << std::endl;

        nodeMap[sourceNode]++;
    }

    //insert nodes with strictly one child in result
    for(std::map<FrameId, unsigned int>::iterator it = nodeMap.begin(); it != nodeMap.end(); it++)
    {
        //std::cout << it->first << " : " << std::to_string(it->second - 1) << " children" << std::endl;
        if(it->second == 1)
        {
            //if rootNode is !NULL remove rootNode from leaves (usefull if the root node has only one child)
            if(!rootNode)
            {
                leafList.push_back(it->first);
            }
            else if(it->first != *rootNode)
            {
                leafList.push_back(it->first);
            }
        }
    }

    return leafList;
}
