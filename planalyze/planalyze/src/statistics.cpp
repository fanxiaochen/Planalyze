#include "color_map.h"
#include "statistics_viewer_widget.h"

#include "statistics.h"

PlotCurve::PlotCurve(const QString &title)
{
  setRenderHint(QwtPlotItem::RenderAntialiased);
  setTitle(title);
}

PlotCurve::~PlotCurve(void)
{

}

void PlotCurve::addSample(int frame, double value)
{
  samples_[frame] = value;

  updateSamples();

  return;
}

void PlotCurve::updateSamples(void)
{
  QVector<QPointF> samples;

  for (std::map<int, double>::iterator it = samples_.begin(); it != samples_.end(); ++ it)
    samples.push_back(QPointF(it->first, it->second));

  setSamples(samples);

  return;
}

Statistics::Statistics(StatisticsViewerWidget* plot)
  :plot_(plot)
{
}

Statistics::~Statistics(void)
{
  for (std::map<int, PlotCurve*>::iterator it = leaf_curves_.begin(); it != leaf_curves_.end(); ++ it)
    delete it->second;
  for (std::map<int, PlotCurve*>::iterator it = stem_curves_.begin(); it != stem_curves_.end(); ++ it)
    delete it->second;
  for (std::map<QString, PlotCurve*>::iterator it = other_curves_.begin(); it != other_curves_.end(); ++ it)
    delete it->second;


  return;
}

void Statistics::addLeafSample(int id, int frame, double value)
{
  std::map<int, PlotCurve*>::iterator it = leaf_curves_.find(id);
  if (it == leaf_curves_.end())
  {
    QString title = QString("Leaf %2").arg(id, 2, 10, QChar('0'));
    PlotCurve* plot_curve = new PlotCurve(title);
    it = leaf_curves_.insert(std::make_pair(id, plot_curve)).first;
    osg::Vec4 color = ColorMap::Instance().getColor(ColorMap::DISCRETE_KEY, id*2+1);
    it->second->setPen(QColor(color.r()*255, color.g()*255, color.b()*255), 2);
    it->second->attach(plot_);
  }

  it->second->addSample(frame, value);
  plot_->replot();

  return;
}

void Statistics::addStemSample(int id, int frame, double value)
{
  std::map<int, PlotCurve*>::iterator it = stem_curves_.find(id);
  if (it == stem_curves_.end())
  {
    QString title = QString("Stem %2").arg(id, 2, 10, QChar('0'));
    PlotCurve* plot_curve = new PlotCurve(title);
    it = leaf_curves_.insert(std::make_pair(id, plot_curve)).first;
    osg::Vec4 color = ColorMap::Instance().getColor(ColorMap::DISCRETE_KEY, id*2);
    it->second->setPen(QColor(color.r()*255, color.g()*255, color.b()*255), 2);
    it->second->attach(plot_);
  }

  it->second->addSample(frame, value);
  plot_->replot();

  return;
}

void Statistics::addOtherSample(const QString& title, int frame, double value)
{
  std::map<QString, PlotCurve*>::iterator it = other_curves_.find(title);
  if (it == other_curves_.end())
  {
    PlotCurve* plot_curve = new PlotCurve(title);
    it = other_curves_.insert(std::make_pair(title, plot_curve)).first;
    osg::Vec4 color = ColorMap::Instance().getColor(ColorMap::DISCRETE_KEY, rand());
    it->second->setPen(QColor(color.r()*255, color.g()*255, color.b()*255), 2);
    it->second->attach(plot_);
  }

  it->second->addSample(frame, value);
  plot_->replot();

  return;
}