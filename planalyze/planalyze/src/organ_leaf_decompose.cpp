#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include "organ.h"
#include "color_map.h"
#include "cgal_types.h"
#include "osg_utility.h"
#include "cgal_utility.h"
#include "parameter_manager.h"

#include "point_cloud.h"

std::vector<std::vector<size_t> > PointCloud::computeLeafComponents(void)
{
  std::vector<std::vector<size_t> > leaf_conponents;
  {
    boost::PlainGraph g_leaf_stem(plant_points_num_);
    initPointGraph(ParameterManager::getInstance().getTriangleLength());
    boost::PointGraph& g_point = *point_graph_;
    typedef boost::PointGraphTraits::edge_iterator edge_iterator;
    std::pair<edge_iterator, edge_iterator> edges = boost::edges(g_point);
    for (edge_iterator it = edges.first; it != edges.second; ++ it)
    {
      size_t source_id = boost::source(*it, g_point);
      size_t target_id = boost::target(*it, g_point);
      if (at(source_id).label != at(target_id).label || at(target_id).label != PclRichPoint::LABEL_LEAF)
        continue;

      boost::add_edge(source_id, target_id, g_leaf_stem);
    }

    std::vector<boost::PlainGraphTraits::vertex_descriptor> component(plant_points_num_);
    size_t component_num = boost::connected_components(g_leaf_stem, &component[0]);

    std::vector<std::vector<size_t> > components(component_num);
    for (size_t i = 0; i < plant_points_num_; ++ i)
      components[component[i]].push_back(i);

    size_t component_size_threshold = ParameterManager::getInstance().getLeafComponentSize();
    for (size_t i = 0; i < component_num; ++ i)
    { 
      if (components[i].size() < component_size_threshold)
      {
        for (size_t j = 0, j_end = components[i].size(); j < j_end; ++ j)
          at(components[i][j]).label = PclRichPoint::LABEL_STEM;
        continue;
      }

      if (at(components[i][0]).label != PclRichPoint::LABEL_LEAF)
        continue;

      leaf_conponents.push_back(components[i]);
    }
  }

  return leaf_conponents;
}

struct CompareComponentBySize
{
  bool operator()(const std::vector<size_t>& a, const std::vector<size_t>& b)
  {
    return a.size() > b.size();
  }
};

void PointCloud::absoluteDetectLeaves(void)
{
  QMutexLocker locker(&mutex_);

  std::cout << "frame: " << getFrame() << "\tabs detecting leaves..." << std::endl;

  leaves_.clear();
  std::vector<std::vector<size_t> > leaf_components = computeLeafComponents();
  std::sort(leaf_components.begin(), leaf_components.end(), CompareComponentBySize());
  for (size_t i = 0, i_end = leaf_components.size(); i < i_end; ++ i)
    addOrgan(leaf_components[i], true);

  std::vector<bool> flags(plant_points_num_, true);
  for (size_t i = 0, i_end = leaf_components.size(); i < i_end; ++ i)
  {
    const std::vector<size_t>& component = leaf_components[i];
    for (size_t j = 0, j_end = component.size(); j < j_end; ++ j)
    {
      flags[component[j]] = false;
    }
  }

  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    if (!flags[i])
      continue;

    if (at(i).label == PclRichPoint::LABEL_LEAF)
      at(i).label = PclRichPoint::LABEL_STEM;
  }

  save(filename_);
  expire();

  return;
}

