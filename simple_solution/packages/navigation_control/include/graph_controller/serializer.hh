#ifndef GRAPH_CONTROLLER__SERIALIZER_HH_
#define GRAPH_CONTROLLER__SERIALIZER_HH_

#include <list>
#include <string>

#include <graph_controller/node.hh>

namespace graph_controller {

class Serializer
{
public:

  std::string SerializeGraph(const std::list<Node> & graph) const;

  std::list<Node> DeSerializeGraph(const std::string & serialized_graph) const;
};

}  // namespace graph_controller

#endif  // GRAPH_CONTROLLER__SERIALIZER_HH_
