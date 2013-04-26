#include <QMenu>
#include <QContextMenuEvent>

#include "main_window.h"
#include "point_cloud.h"
#include "parameter_manager.h"
#include "file_system_model.h"
#include "file_viewer_widget.h"

FileViewerWidget::FileViewerWidget(QWidget * parent)
  : QTreeView(parent),
  model_(new FileSystemModel)
{
  setModel(model_);

  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

  hideColumn(1);
  hideColumn(2);
  hideColumn(3);
}

FileViewerWidget::~FileViewerWidget(void)
{
}

void FileViewerWidget::setWorkspace(const std::string& workspace)
{
  QModelIndex root_index = model_->setRootPath(workspace.c_str());
  setRootIndex(root_index);
  ParameterManager::getInstance().initFrameNumbers();

  return;
}

void FileViewerWidget::contextMenuEvent(QContextMenuEvent *event)
{
  std::string filename = model_->filePath(currentIndex()).toStdString();
  osg::ref_ptr<PointCloud> point_cloud = model_->getPointCloud(filename);
  if (point_cloud == NULL)
    return;

  QMenu menu(this);

  int view = point_cloud->getView();

  if (view != 12)
    menu.addAction("Set Cloud Rotation", point_cloud, SLOT(setRotation()));
  else
  {
    QMenu* stem_menu = menu.addMenu("Initialization");
    stem_menu->addAction("Leaf-Stem Classify", point_cloud, SLOT(absoluteClassify()));
    stem_menu->addAction("Decompose Leaves", point_cloud, SLOT(absoluteDetectLeaves()));
    stem_menu->addAction("Sample Skeleton Points", point_cloud, SLOT(sampleSkeletonPoints()));
    stem_menu->addAction("Initialize Skeleton", point_cloud, SLOT(initializeSkeleton()));
    stem_menu->addAction("Extract Stem Skeleton", point_cloud, SLOT(extractStemSkeleton()));
    stem_menu->addAction("Decompose Stems", point_cloud, SLOT(absoluteDetectStems()));
  }

  QMenu* colorize_menu = menu.addMenu("Colorize Point Cloud");
  colorize_menu->addAction("Original Color", point_cloud, SLOT(setOriginalColor()));
  colorize_menu->addAction("Color by Leaf/Stem", point_cloud, SLOT(setLabelColor()));
  colorize_menu->addAction("Color by Organ", point_cloud, SLOT(setOrganColor()));
  colorize_menu->addAction("Color by Thickness", point_cloud, SLOT(setThicknessColor()));
  colorize_menu->addAction("Color by Flatness", point_cloud, SLOT(setFlatnessColor()));
  colorize_menu->addAction("Color by Segment", point_cloud, SLOT(setSegmentColor()));
  colorize_menu->addAction("Color by Probability", point_cloud, SLOT(setProbabilityColor()));
  colorize_menu->addAction("Uniform Color", point_cloud, SLOT(setUniformColor()));

  QMenu* rendering_menu = menu.addMenu("Toggle Rendering Mode");
  rendering_menu->addAction("Render Plant", point_cloud, SLOT(toggleRenderPlant()));
  rendering_menu->addAction("Render Pot", point_cloud, SLOT(toggleRenderPot()));
  rendering_menu->addAction("Render Noise", point_cloud, SLOT(toggleRenderNoise()));
  rendering_menu->addAction("Render Normals", point_cloud, SLOT(toggleRenderNormals()));
  rendering_menu->addAction("Render Normals", point_cloud, SLOT(toggleRenderOrientations()));

  if (view != 12)
    rendering_menu->addAction("Toggle Draggers", point_cloud, SLOT(toggleDraggers()));
  else
  {
    rendering_menu->addAction("Inner Surface", point_cloud, SLOT(toggleRenderTriangles()));
    rendering_menu->addAction("Primitive Nodes", point_cloud, SLOT(toggleRenderOrgans()));
  }

  if (view != 12)
    menu.addAction(QString("Mark as %1").arg(point_cloud->isRegistered()?("Unregistered"):("Registered")), point_cloud, SLOT(toggleRegisterState()));
  else
  {
    QMenu* file_menu = menu.addMenu("File Load/Save");
    file_menu->addAction("Save Organ Graph", point_cloud, SLOT(saveStatus()));
    file_menu->addAction("Load Organ Graph", point_cloud, SLOT(loadStatus()));
    file_menu->addAction("Save Point Cloud", point_cloud, SLOT(save()));
    menu.addAction("Compute Registration", point_cloud, SLOT(registration()));
  }

  menu.exec(event->globalPos());

  return;
}