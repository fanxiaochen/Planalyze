#pragma once
#ifndef ADAPTER_WIDGET_H
#define ADAPTER_WIDGET_H

#include <QSet>
#include <QEvent>
#include <QMutex>
#include <QQueue>
#include <QGLWidget>
#include <osgViewer/GraphicsWindow>

class QInputEvent;
class ThreadedPainter;

class AdapterWidget : public QGLWidget
{
  typedef QGLWidget inherited;

public:

  AdapterWidget( QWidget* parent = NULL, const QGLWidget* shareWidget = NULL, Qt::WindowFlags f = 0, bool forwardKeyEvents = false );
  AdapterWidget( QGLContext* context, QWidget* parent = NULL, const QGLWidget* shareWidget = NULL, Qt::WindowFlags f = 0, bool forwardKeyEvents = false );
  AdapterWidget( const QGLFormat& format, QWidget* parent = NULL, const QGLWidget* shareWidget = NULL, Qt::WindowFlags f = 0, bool forwardKeyEvents = false );
  virtual ~AdapterWidget();

  osgViewer::GraphicsWindow* getOrganGraphicsWindow() { return _gw.get(); }
  const osgViewer::GraphicsWindow* getOrganGraphicsWindow() const { return _gw.get(); }

  inline bool getForwardKeyEvents() const { return _forwardKeyEvents; }
  virtual void setForwardKeyEvents( bool f ) { _forwardKeyEvents = f; }

  void setKeyboardModifiers( QInputEvent* event );

  virtual void keyPressEvent( QKeyEvent* event );
  virtual void keyReleaseEvent( QKeyEvent* event );
  virtual void mousePressEvent( QMouseEvent* event );
  virtual void mouseReleaseEvent( QMouseEvent* event );
  virtual void mouseDoubleClickEvent( QMouseEvent* event );
  virtual void mouseMoveEvent( QMouseEvent* event );
  virtual void wheelEvent( QWheelEvent* event );

protected:

  int getNumDeferredEvents()
  {
    QMutexLocker lock(&_deferredEventQueueMutex);
    return _deferredEventQueue.count();
  }
  void enqueueDeferredEvent(QEvent::Type eventType, QEvent::Type removeEventType = QEvent::None)
  {
    QMutexLocker lock(&_deferredEventQueueMutex);

    if (removeEventType != QEvent::None)
    {
      if (_deferredEventQueue.removeOne(removeEventType))
        _eventCompressor.remove(eventType);
    }

    if (_eventCompressor.find(eventType) == _eventCompressor.end())
    {
      _deferredEventQueue.enqueue(eventType);
      _eventCompressor.insert(eventType);
    }
  }
  void processDeferredEvents();

  osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _gw;

  QMutex _deferredEventQueueMutex;
  QQueue<QEvent::Type> _deferredEventQueue;
  QSet<QEvent::Type> _eventCompressor;

  bool _forwardKeyEvents;

  friend class ThreadedPainter;

  virtual void resizeEvent( QResizeEvent* event );
  virtual void moveEvent( QMoveEvent* event );
  virtual void glDraw();
  virtual bool event( QEvent* event );
};

#endif // ADAPTER_WIDGET_H