#pragma once
#ifndef STATISTICS_H
#define STATISTICS_H

#include <qwt_plot_curve.h>

class StatisticsViewerWidget;

class PlotCurve : public QwtPlotCurve
{
public:
  PlotCurve(const QString &title);
  virtual ~PlotCurve(void);

  void addSample(int frame, double value);

protected:
  void updateSamples(void);

private:
  std::map<int, double> samples_;
};

class Statistics
{
public:
  Statistics(StatisticsViewerWidget* plot);
  virtual ~Statistics(void);

  void addLeafSample(int id, int frame, double value);
  void addStemSample(int id, int frame, double value);
  void addOtherSample(const QString& title, int frame, double value);
protected:

private:
  std::map<int, PlotCurve*>       leaf_curves_;
  std::map<int, PlotCurve*>       stem_curves_;
  std::map<QString, PlotCurve*>   other_curves_;
  StatisticsViewerWidget*         plot_;

  std::vector<QString>            colors;
};

#endif // STATISTICS_H