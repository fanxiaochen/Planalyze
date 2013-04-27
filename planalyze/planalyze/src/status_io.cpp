#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QMessageBox>
#include <QDomElement>
#include <QDomDocument>
#include <QMutexLocker>

#include <boost/graph/adjacency_list.hpp>

#include "organ.h"
#include "cgal_types.h"
#include "main_window.h"
#include "point_cloud.h"

void PointCloud::loadStatus(void)
{
  MainWindow* main_window = MainWindow::getInstance();

  QString filename = QFileInfo(filename_.c_str()).path()+"/status.xml";
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    //QMessageBox::warning(main_window, "PointCloud::loadStatus", "Couldn't read status file");
    return;
  }

  QDomDocument doc("status");
  if (!doc.setContent(&file)) {
    file.close();
    //QMessageBox::critical(main_window, "PointCloud::loadStatus", "Couldn't read status file");
    return;
  }

  QDomElement root = doc.documentElement();
  //if (root.tagName() != QString("frame_%1").arg(getFrame(), 5, 10, QChar('0'))) {
    //QMessageBox::critical(main_window, "PointCloud::loadStatus", "Unmatch status file");
  //  return;
  //}

  QMutexLocker locker(&mutex_);

  stems_.clear();
  leaves_.clear();
  QDomElement organ_graph = root.firstChildElement("OrganGraph");
  if(!organ_graph.isNull()) {
    int leaf_num = 0;
    int stem_num = 0;
    QDomElement organ_element = organ_graph.firstChildElement("Organ");
    while (!organ_element.isNull()) {
      int organ_id = organ_element.attribute("Id", "").toUInt();
      bool is_leaf = bool(organ_element.attribute("IsLeaf", "").toInt());
      if (is_leaf)
        leaf_num = std::max(leaf_num, organ_id+1);
      else
        stem_num = std::max(stem_num, organ_id+1);
      organ_element = organ_element.nextSiblingElement("Organ");
    }

    if (stem_num != 0)
      stems_.resize(stem_num);

    if (leaf_num)
      leaves_.resize(leaf_num);

    if (leaf_num + stem_num != 0)
    {
      organ_element = organ_graph.firstChildElement("Organ");
      while (!organ_element.isNull()) {
        Organ organ(this, &organ_element);
        if (organ.isLeaf())
          leaves_[organ.getId()] = organ;
        else
          stems_[organ.getId()] = organ;
        organ_element = organ_element.nextSiblingElement("Organ");
      }
    }
  }

  expire();

  return;
}

void PointCloud::saveStatus(void)
{
  MainWindow* main_window = MainWindow::getInstance();

  QString filename = QFileInfo(filename_.c_str()).path()+"/status.xml";
  QFile file(filename);
  if (!file.open(QIODevice::WriteOnly)) {
    QMessageBox::warning(main_window, "PointCloud::saveStatus", "Couldn't write status file");
    return;
  }

  QDomDocument doc("status");
  QDomProcessingInstruction xml_declaration = doc.createProcessingInstruction("xml", "version=\"1.0\"");
  doc.appendChild(xml_declaration);

  QDomElement root = doc.createElement(QString("frame_%1").arg(getFrame(), 5, 10, QChar('0')));
  doc.appendChild(root);

  QDomElement organ_graph = doc.createElement("OrganGraph");
  root.appendChild(organ_graph);

  for (size_t i = 0, i_end = stems_.size(); i < i_end; ++ i)
  {
    QDomElement organ_element = doc.createElement("Organ");
    stems_[i].save(&doc, &organ_element);
    organ_graph.appendChild(organ_element);
  }
  for (size_t i = 0, i_end = leaves_.size(); i < i_end; ++ i)
  {
    QDomElement organ_element = doc.createElement("Organ");
    leaves_[i].save(&doc, &organ_element);
    organ_graph.appendChild(organ_element);
  }

  QTextStream text_stream(&file);
  text_stream << doc.toString();
  file.close();

  return;
}

