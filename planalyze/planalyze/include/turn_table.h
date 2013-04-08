#pragma once
#ifndef TURN_TABLE_H_
#define TURN_TABLE_H_

#include <QString>
#include <QObject>
#include "parameter.h"

class QextSerialPort;

class TurnTable : public QObject
{
  Q_OBJECT

public:
  TurnTable(void);
  ~TurnTable(void);

  bool init(void);

public slots:
  void rotate(double time);

public:
  EnumParameter<std::string>    com_port_;
  IntParameter                  view_number_;

private:
  QextSerialPort    *serial_port_;
};


#endif /*TURN_TABLE_H_*/
