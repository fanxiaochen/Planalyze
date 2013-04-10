#include <QFileDialog>
#include <QTextStream>
#include <QResizeEvent>

#include <osg/Depth>
#include <osg/Point>
#include <osg/LineWidth>
#include <osgDB/WriteFile>
#include <osg/AnimationPath>
#include <osgGA/TrackballManipulator>

#include "parameter.h"
#include "registrator.h"
#include "main_window.h"
#include "light_source.h"
#include "update_visitor.h"
#include "povray_visitor.h"
#include "toggle_handler.h"
#include "snapshot_handler.h"
#include "parameter_dialog.h"
#include "stateset_manipulator.h"
#include "osg_viewer_widget.h"


OSGViewerWidget::OSGViewerWidget(QWidget * parent, const QGLWidget * shareWidget, Qt::WindowFlags f)
  :AdapterWidget(parent, shareWidget, f, true),
  scene_root_(new osg::Group),
  other_root_(new osg::Group),
  gl_thread_(this),
  threaded_painter_(this)
{
  osg::ref_ptr<osg::Group> root = new osg::Group;
  root->addChild(scene_root_);
  root->addChild(other_root_);
  setSceneData(root);

  scene_root_->getOrCreateStateSet()->setAttribute(new osg::Point(2.0f), osg::StateAttribute::ON);
  scene_root_->getOrCreateStateSet()->setAttribute(new osg::LineWidth(1.0f), osg::StateAttribute::ON);

  setCameraManipulator(new osgGA::TrackballManipulator);

  addEventHandler(new osgViewer::HelpHandler);
  addEventHandler(new osgViewer::StatsHandler);
  addEventHandler(new osgViewer::LODScaleHandler);
  addEventHandler(new osgViewer::ThreadingHandler);
  addEventHandler(new osgViewer::ScreenCaptureHandler(new WriteToFile));
  addEventHandler(new RecordCameraPathHandler);

  for (int i = 0; i < 6; ++ i)
  {
    osg::ref_ptr<LightSource> light_source = new LightSource(i);
    light_sources_.push_back(light_source);
    char index = '0'+i;
    addEventHandler(new ToggleHandler(light_source, index, std::string("Toggle Light Source ")+index));
    addChild(light_source, false);
  }

  double w = width();
  double h = height();
  getCamera()->setViewport(new osg::Viewport(0, 0, w, h));
  getCamera()->setProjectionMatrixAsPerspective(60.0f, w/h, 1.0f, 10000.0f);
  getCamera()->setGraphicsContext(getOrganGraphicsWindow());
  getCamera()->setClearColor(osg::Vec4(1, 1, 1, 1.0));

  setThreadingModel(osgViewer::Viewer::SingleThreaded);

  this->doneCurrent();
}

OSGViewerWidget::~OSGViewerWidget()
{
  osgViewer::View::EventHandlers handlers = getEventHandlers();
  for (osgViewer::View::EventHandlers::iterator it = handlers.begin(); it != handlers.end(); ++ it)
    this->removeEventHandler(*it);

  stopRendering();
}

void OSGViewerWidget::increaseLineWidth(void)
{
  osg::StateSet* state_Set = scene_root_->getOrCreateStateSet();
  osg::LineWidth* line_width = dynamic_cast<osg::LineWidth*>(state_Set->getAttribute(osg::StateAttribute::LINEWIDTH));
  if (line_width == NULL)
    return;

  if (line_width->getWidth() >= 16.0)
  {
    line_width->setWidth(16.0);
    return;
  }

  line_width->setWidth(line_width->getWidth()+1.0);
  return;
}

void OSGViewerWidget::decreaseLineWidth(void)
{
  osg::StateSet* state_Set = scene_root_->getOrCreateStateSet();
  osg::LineWidth* line_width = dynamic_cast<osg::LineWidth*>(state_Set->getAttribute(osg::StateAttribute::LINEWIDTH));
  if (line_width == NULL)
    return;

  if (line_width->getWidth() <= 1.0)
  {
    line_width->setWidth(1.0);
    return;
  }

  line_width->setWidth(line_width->getWidth()-1.0);
  return;
}

void OSGViewerWidget::increasePointSize(void)
{
  osg::StateSet* state_Set = scene_root_->getOrCreateStateSet();
  osg::Point* point = dynamic_cast<osg::Point*>(state_Set->getAttribute(osg::StateAttribute::POINT));
  if (point == NULL)
    return;

  if (point->getSize() >= 16.0)
  {
    point->setSize(16.0);
    return;
  }

  point->setSize(point->getSize()+1.0);
  return;
}

void OSGViewerWidget::decreasePointSize(void)
{
  osg::StateSet* state_Set = scene_root_->getOrCreateStateSet();
  osg::Point* point = dynamic_cast<osg::Point*>(state_Set->getAttribute(osg::StateAttribute::POINT));
  if (point == NULL)
    return;

  if (point->getSize() <= 1.0)
  {
    point->setSize(1.0);
    return;
  }

  point->setSize(point->getSize()-1.0);
  return;
}

void OSGViewerWidget::startRendering()
{
  addEventHandler(new StateSetManipulator(getSceneData()->getOrCreateStateSet()));

  threaded_painter_.moveToThread(&gl_thread_);
  connect(&gl_thread_, SIGNAL(started()), &threaded_painter_, SLOT(start()));
  gl_thread_.start();
}

void OSGViewerWidget::stopRendering()
{
  threaded_painter_.stop();
  gl_thread_.wait();
}

void OSGViewerWidget::paintGL(void)
{
  QMutexLocker locker(&mutex_);
  frame();
}

void OSGViewerWidget::resizeEvent(QResizeEvent *event)
{
  threaded_painter_.resizeEvent(event);
}

void OSGViewerWidget::paintEvent(QPaintEvent * /*event*/)
{
  // Handled by the GLThread.
}

void OSGViewerWidget::closeEvent(QCloseEvent *event)
{
  stopRendering();
  QGLWidget::closeEvent(event);
}

void OSGViewerWidget::centerSceneIfNecessary(void)
{
  bool locked = mutex_.tryLock();
  scene_root_->accept(UpdateVisitor());
  if (locked)
    mutex_.unlock();

  osg::BoundingSphere bounding_sphere = getBound();
  if (bounding_sphere.center() == osg::Vec3(0, 0, 0))
    return;

  MainWindow::getInstance()->getRegistrator()->init();

  osg::Vec3d eye, center, up;
  getCamera()->getViewMatrixAsLookAt(eye, center, up);

  double c2 = (eye-bounding_sphere.center()).length2();
  osg::Vec3d view_direction = eye - center;
  view_direction.normalize();
  double a = (eye-bounding_sphere.center())*view_direction;
  double distance = std::sqrt(c2-a*a);
  if (distance < 16*bounding_sphere.radius()) // Line and sphere intersect
    return;

  centerSceneImpl();

  return;
}


void OSGViewerWidget::centerScene(void)
{
  QMutexLocker locker(&mutex_);
  scene_root_->accept(UpdateVisitor());

  centerSceneImpl();

  return;
}

void OSGViewerWidget::centerSceneImpl(void)
{
  osgGA::CameraManipulator* camera_manipulator = getCameraManipulator();

  osg::BoundingSphere bounding_sphere = getBound();
  double radius = bounding_sphere.radius();
  osg::Vec3d eye_offset(0.0, 0.0, -2*radius);
  camera_manipulator->setHomePosition(bounding_sphere.center() + eye_offset, bounding_sphere.center(), osg::Vec3d(0.0f,-1.0f,0.0f));
  camera_manipulator->home(0);

  double offset = radius/1.5;
  std::vector<osg::Vec3> offsets;
  offsets.push_back(osg::Vec3(0, 0, -offset));
  offsets.push_back(osg::Vec3(0, 0, offset));
  offsets.push_back(osg::Vec3(-offset, 0, 0));
  offsets.push_back(osg::Vec3(offset, 0, 0));
  offsets.push_back(osg::Vec3(0, -offset, 0));
  offsets.push_back(osg::Vec3(0, offset, 0));
  for (int i = 0; i < 6; ++ i)
    light_sources_[i]->init(bounding_sphere.center() + offsets[i]);

  return;
}

void OSGViewerWidget::replaceChild(osg::Node *old_child, osg::Node *new_child, bool in_scene)
{
  QMutexLocker locker(&mutex_);
  if (in_scene)
  {
    scene_root_->removeChild(old_child);
    scene_root_->addChild(new_child);
  }
  else
  {
    other_root_->removeChild(old_child);
    other_root_->addChild(new_child);
  }

  centerSceneIfNecessary();

  return;
}

void OSGViewerWidget::addChild(osg::Node *child, bool in_scene)
{
  QMutexLocker locker(&mutex_);
  if (in_scene)
    scene_root_->addChild(child);
  else
    other_root_->addChild(child);

  centerSceneIfNecessary();

  return;
}

void OSGViewerWidget::removeChild(osg::Node *child, bool in_scene)
{
  QMutexLocker locker(&mutex_);
  if (in_scene)
    scene_root_->removeChild(child);
  else
    other_root_->removeChild(child);

  return;
}

osg::BoundingSphere OSGViewerWidget::getBound(void) const
{
  return scene_root_->computeBound();
}

void OSGViewerWidget::removeChildren(bool in_scene)
{
  QMutexLocker locker(&mutex_);
  if (in_scene)
    scene_root_->removeChildren(0, scene_root_->getNumChildren());
  else
    other_root_->removeChildren(0, other_root_->getNumChildren());

  return;
}

void OSGViewerWidget::savePOVRaySnapshot(void)
{
  QString folder = QFileDialog::getExistingDirectory(MainWindow::getInstance(),
    tr("Save POV-Ray Files"), MainWindow::getInstance()->getWorkspace());

  if (folder.isEmpty())
    return;

  QMutexLocker locker(&mutex_);
  PovRayVisitor povray_visitor(this);
  scene_root_->accept(povray_visitor);
  locker.unlock();

  povray_visitor.save(folder);

  MainWindow::getInstance()->updateStatusMessage("POV-Ray snapshot saved!");

  return;
}

void OSGViewerWidget::savePOVRayAnimation(void)
{
  IntParameter frame_numer("Frame Number", "Frame Number", 128, 4, 4096);
  ParameterDialog parameter_dialog(std::string(className())+" Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(&frame_numer);
  if (parameter_dialog.exec() != QDialog::Accepted)
    return;

  QString folder = QFileDialog::getExistingDirectory(MainWindow::getInstance(),
    tr("Save POV-Ray Files"), MainWindow::getInstance()->getWorkspace());

  if (folder.isEmpty())
    return;

  osgDB::ifstream path_file((MainWindow::getInstance()->getWorkspace().toStdString()+"/camera.path").c_str());
  if (!path_file.good())
    return;

  osg::ref_ptr<osg::AnimationPath> anmation_path(new osg::AnimationPath);
  anmation_path->read(path_file);

  QMutexLocker locker(&mutex_);
  PovRayVisitor povray_visitor(this);
  scene_root_->accept(povray_visitor);
  locker.unlock();

  povray_visitor.save(folder, anmation_path.get(), (int)(frame_numer));

  MainWindow::getInstance()->updateStatusMessage("POV-Ray animation saved!");

  return;
}

void OSGViewerWidget::saveSceneAsObj(void)
{
  MainWindow* main_window = MainWindow::getInstance();
  QString filename = QFileDialog::getSaveFileName(main_window, "Save Scene as .obj", main_window->getWorkspace(), "Object File(*.obj)");
  if (filename.isEmpty())
    return;

  osgDB::writeObjectFile(*scene_root_, filename.toStdString());

  return;
}

void OSGViewerWidget::saveCameraParameters(void)
{
  MainWindow* main_window = MainWindow::getInstance();
  QString filename = QFileDialog::getSaveFileName(main_window, "Save Camera Parameters", main_window->getWorkspace(), "Camera Paramters(*.txt)");
  if (filename.isEmpty())
    return;

  filename = QFileInfo(filename).path() + "/" + QFileInfo(filename).baseName();

  osg::Vec3 eye, center, up;
  getCamera()->getViewMatrixAsLookAt(eye, center, up);

  QFile txt_file(filename+".txt");
  txt_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream txt_file_stream(&txt_file);
  txt_file_stream << eye.x() << " " << eye.y() << " " << eye.z() << "\n";
  txt_file_stream << center.x() << " " << center.y() << " " << center.z() << "\n";
  txt_file_stream << up.x() << " " << up.y() << " " << up.z() << "\n";

  QFile inc_file(filename+".inc");
  inc_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream inc_file_stream(&inc_file);
  inc_file_stream << QString("\tcamera\n\t{\n\t    perspective\n\t    up -y\n\t    right x*image_width/image_height\n\t %1\t %2\t %3\t}\n")
    .arg(QString("   location <%1, %2, %3>\n").arg(eye[0]).arg(eye[1]).arg(eye[2]))
    .arg(QString("   sky <%1, %2, %3>\n").arg(up[0]).arg(up[1]).arg(up[2]))
    .arg(QString("   look_at <%1, %2, %3>\n").arg(center[0]).arg(center[1]).arg(center[2]));

  return;
}

void OSGViewerWidget::loadCameraParameters(void)
{
  MainWindow* main_window = MainWindow::getInstance();
  QString filename = QFileDialog::getOpenFileName(main_window, "Load Camera Parameters", main_window->getWorkspace(), "Camera Parameters (*.txt)");
  if (filename.isEmpty())
    return;

  QFile txt_file(filename);
  txt_file.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream txt_file_stream(&txt_file);

  osg::Vec3d eye, center, up;
  txt_file_stream >> eye.x() >> eye.y() >> eye.z();
  txt_file_stream >> center.x() >> center.y() >> center.z();
  txt_file_stream >> up.x() >> up.y() >> up.z();
  getCamera()->setViewMatrixAsLookAt(eye, center, up);

  osgGA::CameraManipulator* camera_manipulator = getCameraManipulator();
  osg::Vec3d e, c, u;
  camera_manipulator->getHomePosition(e, c, u);

  camera_manipulator->setHomePosition(eye, center, up);
  camera_manipulator->home(0);
                             
  camera_manipulator->setHomePosition(e, c, u);

  return;
}