#include "pcl_wrapper_types.h"
#include "pcl_wrapper_exports.h"

#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/organized.h>
#include <pcl/search/impl/organized.hpp>

#include <pcl/registration/lum.h>
#include <pcl/registration/impl/lum.hpp>

#include <pcl/registration/icp.h>
#include <pcl/registration/impl/icp.hpp>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/impl/correspondence_estimation.hpp>

template class PCL_WRAPPER_EXPORTS pcl::IterativeClosestPointWithNormals<PclRichPoint, PclRichPoint>;
template class PCL_WRAPPER_EXPORTS pcl::IterativeClosestPoint<PclPoint, PclPoint>;

template class PCL_WRAPPER_EXPORTS pcl::NormalEstimation<PclRichPoint, PclRichPoint>;

template class PCL_WRAPPER_EXPORTS pcl::KdTreeFLANN<PclRichPoint>;
template class PCL_WRAPPER_EXPORTS pcl::KdTreeFLANN<PclPoint>;

template class PCL_WRAPPER_EXPORTS pcl::search::KdTree<PclRichPoint>;
template class PCL_WRAPPER_EXPORTS pcl::search::KdTree<PclPoint>;

template class PCL_WRAPPER_EXPORTS pcl::search::OrganizedNeighbor<PclRichPoint>;
template class PCL_WRAPPER_EXPORTS pcl::search::OrganizedNeighbor<PclPoint>;

template class PCL_WRAPPER_EXPORTS pcl::registration::LUM<PclRichPoint>;
template class PCL_WRAPPER_EXPORTS pcl::registration::LUM<PclPoint>;

template class PCL_WRAPPER_EXPORTS pcl::registration::CorrespondenceEstimation<PclRichPoint, PclRichPoint>;
template class PCL_WRAPPER_EXPORTS pcl::registration::CorrespondenceEstimation<PclPoint, PclPoint>;