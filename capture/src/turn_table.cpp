#include <iostream>
#include "turn_table.h"
#include "qextserialport.h"

TurnTable::TurnTable(void)
  : serial_port_(NULL),
  com_port_("COM Port", "COM port to the turn table", "COM4", std::map<std::string, std::string>()),
  view_number_("View Number", "How many views to form a complete frame", 12, 6, 36, 6)
{
  std::map<std::string, std::string> port_map;
  port_map["COM1"] = "COM1";
  port_map["COM2"] = "COM2";
  port_map["COM3"] = "COM3";
  port_map["COM4"] = "COM4";
  port_map["COM5"] = "COM5";
  port_map["COM6"] = "COM6";
  com_port_.setCadidates(port_map);
}

TurnTable::~TurnTable(void)
{
}

bool TurnTable::init(void)
{
  delete serial_port_;

  serial_port_ = new QextSerialPort(std::string(com_port_).c_str(), QextSerialPort::Polling);
  serial_port_->setBaudRate(BAUD9600);
  serial_port_->setFlowControl(FLOW_OFF);
  serial_port_->setParity(PAR_NONE);
  serial_port_->setDataBits(DATA_8);
  serial_port_->setStopBits(STOP_1);
  serial_port_->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

  QString acceleration("AC5\x0D");
  serial_port_->write(acceleration.toAscii(),acceleration.size());
  QString deceleration("DE5\x0D");
  serial_port_->write(deceleration.toAscii(),deceleration.size());
  QString speed("VE15\x0D");
  serial_port_->write(speed.toAscii(),speed.size());

  return true;
}

void TurnTable::rotate(double time)
{
  if(time < 4000)
    std::cout << "Warning:\tthere's no enough time for rotation!" << std::endl;

  double angle = 360.0/view_number_;

  // start rotate
  // TODO: set speed to make sure the rotation finished in given time
  QString str(QString("FL%1\x0D").arg(angle*10000));
  serial_port_->write(str.toAscii(),str.size());
}