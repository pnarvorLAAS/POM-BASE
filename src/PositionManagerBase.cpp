#include <infuse_pom_base/PositionManagerBase.hpp>

using namespace PositionManager;

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
