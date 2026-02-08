/**
 * This file is part of Traintastic,
 * see <https://github.com/traintastic/traintastic>.
 *
 * Copyright (C) 2025 Reinder Feenstra
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "simulatorview.hpp"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QHelpEvent>
#include <cmath>

#include <QPainter>
#include <QPainterPath>
#include <QFont>

#include <QToolTip>
#include <QGuiApplication>

#include <QDir>
#include <QFileInfo>

#include <QMenu>
#include <QClipboard>

#include <QVector>

#include "trainsmodel.h"

#include "addtraindialog.h"

#include <QPointer>

#include <QMessageBox>

namespace
{

struct ColorF
{
  float red;
  float green;
  float blue;
};

inline static const std::array<ColorF, 17> colors{{
    {0.00f, 0.00f, 0.00f}, //	None
    {0.00f, 0.00f, 0.00f}, //	Black
    {0.75f, 0.75f, 0.75f}, //	Silver
    {0.50f, 0.50f, 0.50f}, //	Gray
    {1.00f, 1.00f, 1.00f}, // White
    {0.50f, 0.00f, 0.00f}, // Maroon
    {1.00f, 0.00f, 0.00f}, // Red
    {0.50f, 0.00f, 0.50f}, // Purple
    {1.00f, 0.00f, 1.00f}, // Fuchsia
    {0.00f, 0.50f, 0.00f}, // Green
    {0.00f, 1.00f, 0.00f}, // Lime
    {0.50f, 0.50f, 0.00f}, // Olive
    {1.00f, 1.00f, 0.00f}, // Yellow
    {0.00f, 0.00f, 0.50f}, // Navy
    {0.00f, 0.00f, 1.00f}, // Blue
    {0.00f, 0.50f, 0.50f}, // Teal
    {0.00f, 1.00f, 1.00f}, // Aqua
                                                  }};

float crossProduct(Simulator::Point p1, Simulator::Point p2, Simulator::Point p3)
{
  return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
}

bool isPointInTriangle(std::span<const Simulator::Point, 3> triangle, const Simulator::Point point)
{
  const float cross1 = crossProduct(triangle[0], triangle[1], point);
  const float cross2 = crossProduct(triangle[1], triangle[2], point);
  const float cross3 = crossProduct(triangle[2], triangle[0], point);

  const bool hasNeg = (cross1 < 0) || (cross2 < 0) || (cross3 < 0);
  const bool hasPos = (cross1 > 0) || (cross2 > 0) || (cross3 > 0);

  return !(hasNeg && hasPos);
}

bool lineContains(const QPointF& pos,
                  const QPointF& a, const QPointF& b,
                  float &distanceOut,
                  const float tolerance = 1.0)
{
  QPointF topLeft = a, bottomRight = b;
  if(topLeft.x() > bottomRight.x())
    std::swap(topLeft.rx(), bottomRight.rx());
  if(topLeft.y() < bottomRight.y())
    std::swap(topLeft.ry(), bottomRight.ry());


  if(std::abs(a.y() - b.y()) < 0.0001)
  {
    // Horizontal
    const float yDist = std::abs(a.y() - pos.y());
    const float leftDist = topLeft.x() - pos.x();
    const float rightDist = pos.x() - bottomRight.x();
    if(yDist < tolerance &&
        (topLeft.x() - tolerance) <= pos.x() && (bottomRight.x() + tolerance) >= pos.x())
    {
      const float minOutDist = -std::min(leftDist, rightDist);
      distanceOut = yDist;
      if(minOutDist > yDist)
        distanceOut = minOutDist;
      return true;
    }

    return false;
  }

  if(std::abs(a.x() - b.x()) < 0.0001)
  {
    // Vertical
    const float xDist = std::abs(a.x() - pos.x());
    const float leftDist = pos.y() - topLeft.y();
    const float rightDist = bottomRight.y() - pos.y();
    if(xDist < tolerance &&
        (bottomRight.y() - tolerance) <= pos.y() && (topLeft.y() + tolerance) >= pos.y())
    {
      const float minOutDist = -std::min(leftDist, rightDist);
      distanceOut = xDist;
      if(minOutDist > xDist)
        distanceOut = minOutDist;
      return true;
    }

    return false;
  }

  // Diagonal
  if((bottomRight.x() - topLeft.x()) <= (topLeft.y() - bottomRight.y()))
  {
    // Line is near to vertical
    const float resultingX  = a.x() + (pos.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
    distanceOut = std::abs(resultingX - pos.x());
    return distanceOut <= tolerance;
  }

  // Line is near to horizontal
  const float resultingY  = a.y() + (pos.x() - a.x()) * (b.y() - a.y()) / (b.x() - a.x());
  distanceOut = std::abs(resultingY - pos.y());
  return distanceOut <= tolerance;
}

size_t getSegmentAt(const Simulator::Point &point, const Simulator::StaticData &data)
{
  size_t bestIdx = Simulator::invalidIndex;
  float bestDistance = 0.0;

  for (size_t idx = 0; idx < data.trackSegments.size(); idx++)
  {
    const auto &segment = data.trackSegments.at(idx);

    switch (segment.type)
    {
    case Simulator::TrackSegment::Type::Turnout:
    case Simulator::TrackSegment::Type::TurnoutCurved:
    case Simulator::TrackSegment::Type::Turnout3Way:
    {
      std::span<const Simulator::Point, 3> points(
          {segment.points[0], segment.points[1], segment.points[2]});

      if (isPointInTriangle(points, point))
        return idx;

      if (segment.type == Simulator::TrackSegment::Type::Turnout3Way)
      {
        // Check also other curve
        const Simulator::Point arr[3] = {segment.points[0],
                                         segment.points[1],
                                         segment.points[3]};
        if (isPointInTriangle(arr, point))
          return idx;
      }

      continue;
    }
    case Simulator::TrackSegment::Type::Straight:
    {
      QPointF pos(point.x, point.y);

      QPointF a(segment.points[0].x, segment.points[0].y);
      QPointF b(segment.points[1].x, segment.points[1].y);

      QRectF br;
      br.setTop(segment.points[0].y);
      br.setLeft(segment.points[0].x);
      br.setBottom(segment.points[1].y);
      br.setRight(segment.points[1].x);
      br = br.normalized();

      const QRectF brAdj = br.adjusted(-5, -5, 5, 5);

      if (br.width() > 0.0001 && br.height() > 0.0001 && !brAdj.contains(pos))
        continue;

      float segDistance = 0;
      if (!lineContains(pos, a, b, segDistance, 5))
        continue;

      if (bestIdx == Simulator::invalidIndex || segDistance < bestDistance)
      {
        bestIdx = idx;
        bestDistance = segDistance;
      }
      continue;
    }
    case Simulator::TrackSegment::Type::Curve:
    {
      const Simulator::Point center = segment.curves[0].center;
      const Simulator::Point diff = point - center;
      const float radius = std::sqrt(diff.x * diff.x + diff.y * diff.y);
      const float distance = std::abs(radius - segment.curves[0].radius);

      if (distance > 5)
        continue;

      if (bestIdx != Simulator::invalidIndex && distance > bestDistance)
        continue;

      // Y coordinate is swapped
      float angle = std::atan2(-diff.y, diff.x);

      float rotation = segment.rotation;
      if (rotation < 0)
        rotation += 2 * pi;

      const float curveAngle = segment.curves[0].angle;
      float angleMax = -rotation + pi / 2.0 * (curveAngle > 0 ? 1 : -1);
      float angleMin = -rotation - curveAngle + pi / 2.0 * (curveAngle > 0 ? 1 : -1);

      if (curveAngle < 0)
        std::swap(angleMin, angleMax);

      if (angleMin < 0)
      {
        angleMin += 2 * pi;
        angleMax += 2 * pi;
      }

      if (angleMin < 0 && angle < 0)
      {
        angleMin += 2 * pi;
        angleMax += 2 * pi;
      }

      if (angle < 0)
      {
        angle += 2 * pi;
      }

      // TODO: really ugly...
      if (angleMin <= angleMax)
      {
        // min -> max
        if (angle < angleMin || angle > angleMax)
        {
          // Try again with + 2 * pi
          const float angleBis = angle + 2 * pi;
          if (angleBis < angleMin || angleBis > angleMax)
          {
            // Try again with - 2 * pi
            const float angleTer = angle - 2 * pi;
            if (angleTer < angleMin || angleTer > angleMax)
            {
              continue;
            }
          }
        }
      }
      else
      {
        // 0 -> min, max -> 2 * pi
        if (angle > angleMin && angle < angleMax)
        {
          // Try again with + 2 * pi
          const float angleBis = angle + 2 * pi;
          if (angleBis > angleMin && angleBis < angleMax)
          {
            // Try again with - 2 * pi
            const float angleTer = angle - 2 * pi;
            if (angleTer > angleMin && angleTer < angleMax)
            {
              continue;
            }
          }
        }
      }

      bestIdx = idx;
      bestDistance = distance;
      continue;
    }
    default:
      break;
    }
  }

  return bestIdx;
}

float getSegmentPosAt(size_t segmentIndex, const Simulator::Point &point, const Simulator::StaticData &data)
{
  const auto &segment = data.trackSegments.at(segmentIndex);

  switch (segment.type)
  {
  case Simulator::TrackSegment::Type::Straight:
  case Simulator::TrackSegment::Type::Turnout:
  case Simulator::TrackSegment::Type::Turnout3Way:
  {
    float pos = 0.0;

    QRectF br;
    br.setTop(segment.points[0].y);
    br.setLeft(segment.points[0].x);
    br.setBottom(segment.points[1].y);
    br.setRight(segment.points[1].x);
    br = br.normalized();

    if(br.width() >= br.height())
    {
      if(point.x <= br.left())
        pos = 0.0;
      else if(point.x >= br.right() || br.width() < 0.0001)
        pos = 0.9999; // 1.0
      else
        pos = (point.x - br.left()) / br.width();

      if(segment.points[0].x == br.right())
        pos = 1 - pos;
    }
    else
    {
      if(point.y <= br.top())
        pos = 0.0;
      else if(point.y >= br.bottom() || br.height() < 0.0001)
        pos = 0.9999; // 1.0
      else
        pos = (point.y - br.top()) / br.height();

      if(segment.points[0].y == br.bottom())
        pos = 1 - pos;
    }

    return pos * segment.straight.length;
  }
  case Simulator::TrackSegment::Type::Curve:
  case Simulator::TrackSegment::Type::TurnoutCurved:
  {
    const Simulator::Point center = segment.curves[0].center;
    const Simulator::Point diff = point - center;

    // Y coordinate is swapped
    float angle = std::atan2(-diff.y, diff.x);

    float rotation = segment.rotation;
    if (rotation < 0)
      rotation += 2 * pi;

    const float curveAngle = segment.curves[0].angle;
    float angleMax = -rotation + pi / 2.0 * (curveAngle > 0 ? 1 : -1);
    float angleMin = -rotation - curveAngle + pi / 2.0 * (curveAngle > 0 ? 1 : -1);

    if (curveAngle < 0)
      std::swap(angleMin, angleMax);

    if (angleMin < 0)
    {
      angleMin += 2 * pi;
      angleMax += 2 * pi;
    }

    if (angleMin < 0 && angle < 0)
    {
      angleMin += 2 * pi;
      angleMax += 2 * pi;
    }

    if (angle < 0)
    {
      angle += 2 * pi;
    }

    // TODO: really ugly...
    if (angleMin <= angleMax)
    {
      // min -> max
      if (angle < angleMin || angle > angleMax)
      {
        // Try again with + 2 * pi
        const float angleBis = angle + 2 * pi;
        if (angleBis < angleMin || angleBis > angleMax)
        {
          // Try again with - 2 * pi
          const float angleTer = angle - 2 * pi;
          if (angleTer < angleMin || angleTer > angleMax)
          {
            return 0;
          }

          angle = angleTer;
        }
        else
        {
          angle = angleBis;
        }
      }

      angle = angleMax - angle;
    }
    else
    {
      // 0 -> min, max -> 2 * pi
      if (angle > angleMin && angle < angleMax)
      {
        // Try again with + 2 * pi
        const float angleBis = angle + 2 * pi;
        if (angleBis > angleMin && angleBis < angleMax)
        {
          // Try again with - 2 * pi
          const float angleTer = angle - 2 * pi;
          if (angleTer > angleMin && angleTer < angleMax)
          {
            return 0;

            angle = angleTer;
          }
          else
          {
            angle = angleBis;
          }
        }

        angle = angleMax - angle;
      }
    }

    // Invert start reference
    if(curveAngle < 0)
      angle = -curveAngle - angle;

    return std::clamp(angle * segment.curves[0].radius, 0.0f, segment.curves[0].length);
  }
  default:
    break;
  }

  return 0.0f;
}

void drawStraight(const Simulator::TrackSegment& segment, QPainter *painter)
{
  painter->drawLine(QPointF(0, 0), QPointF(segment.straight.length, 0));
}

void drawCurve(const Simulator::TrackSegment& segment, size_t curveIndex, QPainter *painter)
{
  const auto& curve = segment.curves[curveIndex];
  const float rotation = curve.angle < 0 ? 0.0f : pi;

  int numSegments = qCeil(curve.length / 10.0); // Smooth curve
  float step = curve.angle / numSegments;
  const float cx = curve.radius * sinf(rotation);
  const float cy = curve.radius * -cosf(rotation);

  QVector<QPointF> pointVec;
  pointVec.reserve(numSegments + 1);
  pointVec.append({0, 0});

  for(int i = 1; i <= numSegments; i++)
  {
    float angle = rotation + i * step;
    float x = cx - curve.radius * sinf(angle);
    float y = cy - curve.radius * -cosf(angle);
    pointVec.append({x, y});
  }

  painter->drawPolyline(pointVec.data(), pointVec.size());
}

}

SimulatorView::SimulatorView(QWidget* parent)
  : QWidget(parent)
{
  setFocusPolicy(Qt::StrongFocus); // for key stuff

  // 800 ms turnout blink
  turnoutBlinkTimer.start(std::chrono::milliseconds(800), Qt::PreciseTimer, this);

  setContextMenuPolicy(Qt::DefaultContextMenu);

  setMouseTracking(true);

  mTrainsModel = new TrainsModel(this);
  connect(mTrainsModel, &TrainsModel::setCurrentTrain,
          this, &SimulatorView::doSetCurrentTrainIndex);

  // Dark gray background
  QPalette p = palette();
  p.setColor(QPalette::Window, QColor(70, 70, 70));
  setPalette(p);
  setAutoFillBackground(true);
}

SimulatorView::~SimulatorView()
{
  setSimulator({}, false, false);

  delete mTrainsModel;
  mTrainsModel = nullptr;
}

Simulator* SimulatorView::simulator() const
{
  return m_simulator.get();
}

void SimulatorView::setSimulator(std::shared_ptr<Simulator> value,
                                 bool localOnly, bool discoverable)
{
  if(m_simulator)
  {
    m_simulatorConnections.clear();
    m_simulator->stop();
  }

  m_simulator = std::move(value);
  m_turnouts.clear();
  m_images.clear();

  mTrainsModel->setSimulator(m_simulator.get());

  if(m_simulator)
  {
    const size_t count = m_simulator->staticData.trackSegments.size();
    for(size_t i = 0; i < count; ++i)
    {
      const auto& segment = m_simulator->staticData.trackSegments[i];
      if(segment.type == Simulator::TrackSegment::Type::Turnout || segment.type == Simulator::TrackSegment::Type::TurnoutCurved)
      {
        m_turnouts.emplace_back(Turnout{i, std::span<const Simulator::Point, 3>(segment.points.data(), 3)});
      }
    }

    m_stateData = m_simulator->stateData();
    emit powerOnChanged(m_stateData.powerOn);

    m_simulatorConnections.emplace_back(m_simulator->onTick.connect(
                                          [this]()
    {
      QMetaObject::invokeMethod(this, &SimulatorView::tick, Qt::QueuedConnection);
    }));
    m_simulatorConnections.emplace_back(m_simulator->onTrainAddedRemoved.connect(
        [this](bool add, size_t trainIdx)
        {
          // NOTE: invoked in simulator working thread
          trainAddedRemoved(add, trainIdx);
        }));

    m_simulator->enableServer(localOnly);

    m_simulator->start(discoverable);

    setSignalsScaleFactor(m_simulator->staticData.defaultSignalScale);

    for(const auto &imgRef : m_simulator->staticData.images)
    {
      Image item;
      item.ref = imgRef;

      if(!item.img.load(QString::fromStdString(item.ref.fileName)))
        continue;

      m_images.push_back(item);

      // const QSizeF imgSize = QSizeF(item.img.size()) * item.ref.ratio;

      // const float cosRotation = std::cos(item.ref.rotation);
      // const float sinRotation = std::sin(item.ref.rotation);
      // Simulator::updateView(m_simulator->staticData.view,
      //                       {item.ref.origin.x + imgSize.width() * cosRotation, item.ref.origin.y + imgSize.width() * sinRotation}); // top right
      // Simulator::updateView(m_simulator->staticData.view,
      //            {item.ref.origin.x - imgSize.height() * sinRotation, item.ref.origin.y + imgSize.height() * cosRotation}); // bottom left
      // Simulator::updateView(m_simulator->staticData.view,
      //            {item.ref.origin.x - imgSize.height() * sinRotation + imgSize.width() * cosRotation, item.ref.origin.y + imgSize.height() * cosRotation + imgSize.width() * sinRotation}); // bottom right
    }
  }

  update();
}

void SimulatorView::loadExtraImages(const nlohmann::json& world,
                                    const QString& imagesFile,
                                    QStringList &namesOut)
{
  m_extraImages.clear();

  const QDir fileDir = QFileInfo(imagesFile).absoluteDir();

  if(auto images = world.find("images"); images != world.end() && images->is_array())
  {
    for(const auto& object : *images)
    {
      if(!object.is_object())
      {
        continue;
      }

      Simulator::ImageRef item;

      item.origin.x = object.value("x", std::numeric_limits<float>::quiet_NaN());
      item.origin.y = object.value("y", std::numeric_limits<float>::quiet_NaN());
      item.fileName = object.value<std::string_view>("file", {});
      item.rotation = deg2rad(object.value("rotation", 0.0f));
      item.opacity = object.value("opacity", 1.0);

      const float pxCount = object.value("n_px", std::numeric_limits<float>::quiet_NaN());
      const float mtCount = object.value("n_mt", std::numeric_limits<float>::quiet_NaN());

      if(!item.origin.isFinite() || item.fileName.empty() || pxCount == 0)
      {
        continue;
      }

      item.ratio = mtCount / pxCount;

      Image img;
      img.ref = item;

      QString fileName = QString::fromStdString(img.ref.fileName);
      QFileInfo info(fileName);
      if(info.isRelative())
      {
        // Treat as relative to image JSON file
        fileName = fileDir.absoluteFilePath(fileName);
      }

      if(!img.img.load(fileName))
        continue;

      m_extraImages.push_back(img);

      namesOut.append(info.fileName());
    }
  }

  update();
}

void SimulatorView::zoomIn()
{
  m_zoomFit = false;
  setZoomLevel(m_zoomLevel * zoomFactorIn);
}

void SimulatorView::zoomOut()
{
  m_zoomFit = false;
  setZoomLevel(m_zoomLevel * zoomFactorOut);
}

void SimulatorView::zoomToFit()
{
  if(!m_simulator) [[unlikely]]
  {
    return;
  }

    m_zoomFit = true;

  // Make it fit:
  const float zoomLevelX = width() / m_simulator->staticData.view.width();
  const float zoomLevelY = height() / m_simulator->staticData.view.height();
  const float zoomLevel = std::min(zoomLevelX, zoomLevelY);

  // Center it:
  m_cameraX = m_simulator->staticData.view.left - (width() / zoomLevel - m_simulator->staticData.view.width()) / 2;
  m_cameraY = m_simulator->staticData.view.top - (height() / zoomLevel - m_simulator->staticData.view.height()) / 2;

  setZoomLevel(zoomLevel);
}

void SimulatorView::setCamera(const Simulator::Point &cameraPt)
{
  m_cameraX = cameraPt.x;
  m_cameraY = cameraPt.y;
  update();
}

void SimulatorView::paintEvent(QPaintEvent */*e*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  painter.scale(m_zoomLevel, m_zoomLevel);
  painter.translate(-m_cameraX, -m_cameraY);

  const QTransform trasf = painter.transform();

  if(!m_images.empty() || !m_extraImages.empty())
  {
    for(const auto &image : m_extraImages)
    {
      if(!image.visible)
        continue;
      painter.setOpacity(image.ref.opacity);
      painter.translate(image.ref.origin.x, image.ref.origin.y);
      painter.rotate(qRadiansToDegrees(image.ref.rotation));
      painter.scale(image.ref.ratio, image.ref.ratio);
      painter.drawImage(QPoint(), image.img);
      painter.setTransform(trasf);
    }

    for(const auto &image : m_images)
    {
      painter.setOpacity(image.ref.opacity);
      painter.translate(image.ref.origin.x, image.ref.origin.y);
      painter.rotate(qRadiansToDegrees(image.ref.rotation));
      painter.scale(image.ref.ratio, image.ref.ratio);
      painter.drawImage(QPoint(), image.img);
      painter.setTransform(trasf);
    }

    painter.setOpacity(1);
  }


  if(m_simulator) [[likely]]
  {
    drawMisc(&painter);
    drawTracks(&painter);
    drawTrackObjects(&painter);
    drawTrains(&painter);
  }
}

void SimulatorView::resizeEvent(QResizeEvent* e)
{
  QWidget::resizeEvent(e);
  if(m_zoomFit)
  {
    zoomToFit();
  }
}

void SimulatorView::drawTracks(QPainter *painter)
{
  assert(m_simulator);

  QPen trackPen(QColor(159, 255, 159), 1.435);
  trackPen.setCapStyle(Qt::FlatCap);

  if(m_thinTracks)
  {
    trackPen.setWidth(1);
    trackPen.setCosmetic(true);
  }

  QPen trackPenOccupied = trackPen;
  trackPenOccupied.setColor(QColor(255, 0, 0));

  QPen trackPenGreen = trackPen;
  trackPenGreen.setColor(QColor(20, 144, 0));

  QPen trackPenPurple = trackPen;
  trackPenPurple.setColor(QColor(128, 0, 255));

  QPen trackPenCyan = trackPen;
  trackPenCyan.setColor(QColor(0, 255, 255));
  trackPenCyan.setWidthF(0.5);
  trackPenCyan.setCosmetic(false);

  if(m_thinTracks)
  {
    trackPenCyan.setWidth(1);
    trackPenCyan.setCosmetic(true);
  }

  QPen trackPenMagenta = trackPenCyan;
  trackPenMagenta.setColor(Qt::magenta);

  const QTransform trasf = painter->transform();

  size_t idx = 0;
  for(const auto& segment : m_simulator->staticData.trackSegments)
  {
    const bool occupied = segment.hasSensor() && m_stateData.sensors[segment.sensor.index].value;
    if(m_showTrackOccupancy && occupied)
    {
      // Red if occupied
      painter->setPen(trackPenOccupied);
    }
    else
    {
      painter->setPen(trackPen);
    }

    if(!m_stateData.powerOn)
    {
      // Red if not powered for better contrast
      painter->setPen(trackPenOccupied);

      if(segment.hasSensor() && segment.sensor.index == m_hoverSensorIdx)
      {
        // Blue on same hovered sensor
        painter->setPen(trackPenPurple);
      }
    }

    if((!m_stateData.powerOn || !occupied) && idx == m_hoverSegmentIdx)
    {
      // Green on hover
      painter->setPen(trackPenGreen);
    }

    painter->translate(segment.origin().x, segment.origin().y);
    painter->rotate(qRadiansToDegrees(segment.rotation));

    if(segment.type == Simulator::TrackSegment::Type::Straight)
    {
      drawStraight(segment, painter);
    }
    else if(segment.type == Simulator::TrackSegment::Type::Curve)
    {
      drawCurve(segment, 0, painter);
    }
    else if(segment.type == Simulator::TrackSegment::Type::Turnout)
    {
      drawCurve(segment, 0, painter);
      drawStraight(segment, painter);
    }
    else if(segment.type == Simulator::TrackSegment::Type::TurnoutCurved)
    {
      drawCurve(segment, 1, painter);
      drawCurve(segment, 0, painter);
    }
    else if(segment.type == Simulator::TrackSegment::Type::Turnout3Way)
    {
      drawCurve(segment, 0, painter);
      drawCurve(segment, 1, painter);
      drawStraight(segment, painter);
    }

    painter->setTransform(trasf);
    idx++;
  }

  // Redraw on top turnout current state
  for(const auto& segment : m_simulator->staticData.trackSegments)
  {
    if(segment.type == Simulator::TrackSegment::Type::Straight ||
       segment.type == Simulator::TrackSegment::Type::Curve)
      continue;

    painter->translate(segment.origin().x, segment.origin().y);
    painter->rotate(qRadiansToDegrees(segment.rotation));

    assert(segment.turnout.index < m_stateData.turnouts.size());
    const auto state = m_stateData.turnouts[segment.turnout.index].state;

    if(!m_stateData.powerOn || (m_showTrackOccupancy && segment.hasSensor() && m_stateData.sensors[segment.sensor.index].value))
    {
      // Cyan contrast on red
      painter->setPen(trackPenCyan);
    }
    else
    {
      // Dark yellow contrast on white
      painter->setPen(trackPenMagenta);
    }

    if(segment.type == Simulator::TrackSegment::Type::Turnout)
    {
      switch(state)
      {
      case Simulator::TurnoutState::State::Unknown:
        // Blink cyan or normal
        if(turnoutBlinkState)
        {
          drawCurve(segment, 0, painter);
          drawStraight(segment, painter);
        }
        break;

      case Simulator::TurnoutState::State::Closed:
        drawStraight(segment, painter);
        break;

      case Simulator::TurnoutState::State::Thrown:
        drawCurve(segment, 0, painter);
        break;

      default:
        assert(false);
        break;
      }
    }
    else if(segment.type == Simulator::TrackSegment::Type::TurnoutCurved)
    {
      switch(state)
      {
      case Simulator::TurnoutState::State::Unknown:
        // Blink cyan or normal
        if(turnoutBlinkState)
        {
          drawCurve(segment, 0, painter);
          drawCurve(segment, 1, painter);
        }
        break;

      case Simulator::TurnoutState::State::Closed:
        drawCurve(segment, 0, painter);
        break;

      case Simulator::TurnoutState::State::Thrown:
        drawCurve(segment, 1, painter);
        break;

      default:
        assert(false);
        break;
      }
    }
    else if(segment.type == Simulator::TrackSegment::Type::Turnout3Way)
    {
      switch(state)
      {
      case Simulator::TurnoutState::State::Unknown:
        // Blink cyan or normal
        if(turnoutBlinkState)
        {
          drawCurve(segment, 0, painter);
          drawCurve(segment, 1, painter);
          drawStraight(segment, painter);
        }
        break;

      case Simulator::TurnoutState::State::Closed:
        drawStraight(segment, painter);
        break;

      case Simulator::TurnoutState::State::ThrownLeft:
        drawCurve(segment, 0, painter);
        break;

      case Simulator::TurnoutState::State::ThrownRight:
        drawCurve(segment, 1, painter);
        break;

      default:
        assert(false);
        break;
      }
    }

    painter->setTransform(trasf);
    idx++;
  }

  if(!m_stateData.powerOn && m_hoverSegmentIdx != Simulator::invalidIndex)
  {
    const Simulator::TrackSegment& segment = m_simulator->staticData.trackSegments[m_hoverSegmentIdx];
    if(segment.type == Simulator::TrackSegment::Type::Curve)
    {
      // Draw hovered curve center on top
      QPen p = trackPenGreen;
      p.setWidthF(8);
      painter->setPen(p);

      const Simulator::Point c = segment.curves[0].center;
      painter->drawPoint(QPointF(c.x, c.y));
    }
  }
}

void SimulatorView::drawTrackObjects(QPainter *painter)
{
  assert(m_simulator);

  QColor positionSensorActive = Qt::red;
  QColor positionSensorInactive = Qt::darkGreen;
  QColor axleCountSensorColor = Qt::cyan;
  QColor axleCountSensorDecrease = Qt::darkYellow;

  // Make signals more visible at low zoom levels by scaling
  QPen signalMastPen(Qt::lightGray, 0.6 * m_signalsScaleFactor);
  signalMastPen.setCapStyle(Qt::FlatCap);

  QPen signalMastPenBlack = signalMastPen;
  signalMastPenBlack.setColor(Qt::black);

  QPen signalMastPenWhite = signalMastPen;
  signalMastPenWhite.setColor(Qt::white);
  signalMastPenWhite.setStyle(Qt::DashLine);
  const qreal dashLength = 0.5 * m_signalsScaleFactor / signalMastPenWhite.widthF();
  signalMastPenWhite.setDashPattern({dashLength, dashLength});

  const QPen signalLightPen(Qt::lightGray, 0.2 * m_signalsScaleFactor);
  const QPen signalTrianglePen(Qt::black, 0.1 * m_signalsScaleFactor);

  QPen signalLightArrowPenOff(Qt::darkGray, 0.2 * m_signalsScaleFactor);
  signalLightArrowPenOff.setJoinStyle(Qt::RoundJoin);

  QPen signalLightArrowPenOn = signalLightArrowPenOff;
  signalLightArrowPenOn.setColor(Qt::white);

  QPen signalIndicatorBorder(Qt::darkGray, 0.1 * m_signalsScaleFactor);

  const qreal mastBaseLength = 3.0 * m_signalsScaleFactor;
  const qreal lightDiameter = 2.0 * m_signalsScaleFactor;
  const qreal triangleEdge = 1.9 * m_signalsScaleFactor;
  const qreal advanceSignalWidth = 1.6 * m_signalsScaleFactor;
  const qreal advanceSignalHeight = 0.8 * m_signalsScaleFactor;
  const qreal directionIndicatorWidth = 1.2 * m_signalsScaleFactor;
  const qreal directionIndicatorHeight = 1.6 * m_signalsScaleFactor;

  QFont triangleFont;
  triangleFont.setBold(true);
  triangleFont.setPointSizeF(triangleEdge * 0.3);

  QFont directionFont;
  directionFont.setBold(true);
  directionFont.setPointSizeF(directionIndicatorWidth * 0.9);

  // Zoom dwarf signals a bit less
  const qreal LightDwarfFactor = 0.4 + 0.6 * m_signalsScaleFactor;

  // Zoom dwarf signals even less because they are quite big
  const qreal RotatingDwarfFactor = 0.6 + 0.4 * m_signalsScaleFactor;

  QPen lightDwarfBorderPen(Qt::darkGray, 0.1 * LightDwarfFactor);
  lightDwarfBorderPen.setJoinStyle(Qt::RoundJoin);

  QPen rotDwarfBorderPenBlack(Qt::darkGray, 0.1 * RotatingDwarfFactor);;
  rotDwarfBorderPenBlack.setColor(Qt::black);

  QPen dwarfDiagPenBlack(Qt::black, 0.2 * RotatingDwarfFactor);

  QPen trackPen(QColor(204, 204, 204), 1);
  trackPen.setCapStyle(Qt::RoundCap);

  const QTransform trasf = painter->transform();

  QPen borderPen(Qt::red, 1);
  const float trainWidth = m_simulator->staticData.trainWidth;

  for(const auto& segment : m_simulator->staticData.trackSegments)
  {
    using Object = Simulator::TrackSegment::Object;

    for(const Object& obj : segment.objects)
    {
      painter->translate(obj.pos.x, obj.pos.y);

      float objRot = obj.rotation;
      if(!obj.dirForward)
        objRot += pi;

      const qreal deg = qRadiansToDegrees(objRot);
      painter->rotate(deg);

      switch (obj.type)
      {
      case Object::Type::PositionSensor:
      {
        QColor color;
        const auto &sensorState = m_stateData.sensors[obj.sensorIndex];

        switch(m_simulator->staticData.sensors[obj.sensorIndex].type)
        {
        case Simulator::Sensor::Type::PositionSensor:
        {
          color = positionSensorInactive;
          if(sensorState.value)
            color = positionSensorActive;
          break;
        }
        case Simulator::Sensor::Type::AxleCounter:
        {
          color = axleCountSensorColor;
          if(sensorState.curTime > 0)
            color = positionSensorActive;
          else if(sensorState.curTime < 0)
            color = axleCountSensorDecrease;
          break;
        }
        default:
          break;
        }

        QRectF r;
        r.setSize(QSizeF(4, 2));
        r.moveCenter(QPointF(0, obj.lateralDiff));
        painter->fillRect(r, color);
        break;
      }
      case Object::Type::MainSignal:
      {
        auto signIt = m_stateData.mainSignals.find(obj.signalName);
        if(signIt == m_stateData.mainSignals.end())
          continue;

        // Draw with Y along mast
        painter->rotate(90.0);

        Simulator::MainSignal *signal = signIt->second;

        // Adjust later placment
        qreal lateralDiff = obj.lateralDiff;
        if(signal->zoomLateralDiff)
          lateralDiff = obj.lateralDiff * (0.6 + 0.4 * m_signalsScaleFactor);

        // Align every signal as if it had 3 lights (std::max() if wrongly specified more than 3 lights)
        qreal mastLength = mastBaseLength + lightDiameter * std::max(signal->lights.size() - 1, size_t(3));
        const size_t numElements = signal->hasAdvanceSignal + signal->hasDirectionIndicator + signal->hasRappel;

        if(numElements > 2)
        {
          mastLength *= 1.3;
        }
        else if(numElements > 1)
        {
          mastLength *= 1.1;
        }

        // Draw black mast with white dashes for pure distant signals
        if(signal->isPureDistantSignal)
          painter->setPen(signalMastPenBlack);
        else
          painter->setPen(signalMastPen);

        painter->drawLine(QLineF(lateralDiff, 0,
                                 lateralDiff, -mastLength));

        if(signal->isPureDistantSignal)
        {
          // Now draw dashes
          painter->setPen(signalMastPenWhite);
          painter->drawLine(QLineF(lateralDiff, - mastBaseLength * 0.7,
                                   lateralDiff, -mastLength));
        }

        QRectF lightRect;
        lightRect.setSize(QSizeF(lightDiameter, lightDiameter));
        lightRect.moveCenter(QPointF(lateralDiff, - mastLength + lightDiameter / 2.0));

        painter->setPen(signalLightPen);

        // Calculate current blink state, keep in range [0, 7]
        int8_t blinkState = int8_t(m_stateData.signalBlinkState) - int8_t(signal->getSignalBlinkStart(false));
        if(blinkState >= 8)
          blinkState -= 8;
        else if(blinkState < 0)
          blinkState += 8;

        // Light 0 is always topmost
        for(size_t i = 0; i < signal->lights.size(); i++)
        {
          if(signal->square)
          {
            // Draw circle light without extra border
            // and with black square background
            painter->setPen(Qt::NoPen);
            painter->setBrush(Qt::black);
            painter->drawRect(lightRect);
          }

          bool on = false;
          switch (signal->lights.at(i).state)
          {
          case Simulator::MainSignal::State::Off:
            on = false;
            break;

          case Simulator::MainSignal::State::On:
            on = true;
            break;

          case Simulator::MainSignal::State::BlikOn:
            // Blink relay start off
            on = blinkState >= 4;
            break;

          case Simulator::MainSignal::State::BlinkReverseOn:
            on = blinkState < 4;
            break;
          default:
            break;
          }

          if(on)
          {
            switch (signal->lights.at(i).color)
            {
            case Simulator::MainSignal::Light::Color::Red:
              painter->setBrush(Qt::red);
              break;
            case Simulator::MainSignal::Light::Color::Yellow:
              painter->setBrush(Qt::yellow);
              break;
            case Simulator::MainSignal::Light::Color::Green:
              painter->setBrush(Qt::green);
              break;
            default:
              break;
            }
          }
          else
          {
            painter->setBrush(Qt::black);
          }

          painter->drawEllipse(lightRect);

          if(signal->square)
          {
            // Now draw square border on top
            painter->setPen(signalLightPen);
            painter->setBrush(Qt::NoBrush);
            painter->drawRect(lightRect);
          }

          // Go down to next light
          lightRect.moveTop(lightRect.top() + lightDiameter);
        }

        if(signal->fixedLimit != Simulator::MainSignal::FixedLimit::NoLimit)
        {
          const QPointF triangle[3] = {
              {lateralDiff - triangleEdge / 2.0, lightRect.top() + triangleEdge * 0.14},
              {lateralDiff + triangleEdge / 2.0, lightRect.top() + triangleEdge * 0.14},
              {lateralDiff, lightRect.top() + triangleEdge}
          };

          // 0.55 to align text a bit above triangle center to better use its larger half
          const QRectF triangleRect(triangle[0].x(), triangle[0].y(),
                                    triangle[1].x() - triangle[0].x(),
                                    (triangle[2].y() - triangle[0].y()) * 0.55);

          painter->setPen(signalTrianglePen);
          painter->setBrush(Qt::white);
          painter->drawConvexPolygon(triangle, 3);

          if(signal->fixedLimit == Simulator::MainSignal::FixedLimit::Limit60)
          {
            painter->setFont(triangleFont);
            painter->setPen(Qt::black);
            painter->setBrush(Qt::NoBrush);
            painter->drawText(triangleRect, Qt::AlignCenter, QLatin1String("60"));
          }

          // Go down to next element
          lightRect.moveTop(lightRect.top() + triangleEdge);
        }

        if(signal->hasAdvanceSignal)
        {
          // Start signal blink is independent from main light blink
          blinkState = int8_t(m_stateData.signalBlinkState) - int8_t(signal->getSignalBlinkStart(true));
          if(blinkState >= 8)
            blinkState -= 8;
          else if(blinkState < 0)
            blinkState += 8;

          QPointF ct(lateralDiff, lightRect.top() + advanceSignalHeight * 1.2);
          QRectF startSignalRect(0, 0, advanceSignalWidth, advanceSignalHeight);
          startSignalRect.moveCenter(ct);

          painter->setBrush(Qt::black);
          painter->setPen(Qt::NoPen);
          painter->drawRoundedRect(startSignalRect, advanceSignalHeight / 2.0, advanceSignalHeight / 2.0);

          bool on = false;
          switch (signal->getAdvanceSignalState())
          {
          case Simulator::MainSignal::State::Off:
            on = false;
            break;

          case Simulator::MainSignal::State::On:
            on = true;
            break;

          case Simulator::MainSignal::State::BlikOn:
            // Blink relay start off
            on = blinkState >= 4;
            break;

          case Simulator::MainSignal::State::BlinkReverseOn:
            on = blinkState < 4;
            break;
          default:
            break;
          }

          painter->setBrush(on ? Qt::white : Qt::darkGray);

          QRectF startSignalLightRect(0, 0, advanceSignalHeight * 0.7, advanceSignalHeight * 0.7);
          startSignalLightRect.moveCenter(QPointF(startSignalRect.left() + advanceSignalHeight / 2.0, ct.y()));
          painter->drawEllipse(startSignalLightRect);

          startSignalLightRect.moveCenter(QPointF(startSignalRect.right() - advanceSignalHeight / 2.0, ct.y()));
          painter->drawEllipse(startSignalLightRect);

          // Go down to next element
          lightRect.moveTop(startSignalRect.bottom());
        }

        if(signal->hasDirectionIndicator)
        {
          QPointF ct(lateralDiff, lightRect.top() + directionIndicatorHeight * 0.7);
          QRectF directionIndicatorRect(0, 0, directionIndicatorWidth, directionIndicatorHeight);
          directionIndicatorRect.moveCenter(ct);

          painter->setBrush(Qt::black);
          painter->setPen(signalIndicatorBorder);
          painter->drawRect(directionIndicatorRect);

          QChar letter = QChar::fromLatin1(signal->directionIndicatorText);
          if(letter.isLetterOrNumber() && letter != ' ')
          {
            painter->setFont(directionFont);
            painter->setPen(Qt::white);
            painter->setBrush(Qt::NoBrush);
            painter->drawText(directionIndicatorRect, Qt::AlignCenter,
                              QChar::fromLatin1(signal->directionIndicatorText));
          }

          // Go down to next element
          lightRect.moveTop(directionIndicatorRect.bottom());
        }

        if(signal->hasRappel)
        {
          QPointF ct(lateralDiff, lightRect.top() + directionIndicatorHeight * 0.7);
          QRectF rappelRect(0, 0, directionIndicatorWidth, directionIndicatorHeight);
          rappelRect.moveCenter(ct);

          QRectF rappelRectAdj = rappelRect.adjusted(signalLightArrowPenOn.widthF(), signalLightArrowPenOn.widthF(),
                                                     -signalLightArrowPenOn.widthF(), -signalLightArrowPenOn.widthF());

          painter->fillRect(rappelRect, Qt::black);

          if(signal->rappelState == Simulator::MainSignal::RappelState::OneLine_60)
          {
            QLineF line(rappelRectAdj.left(), ct.y(), rappelRectAdj.right(), ct.y());
            painter->setPen(signalLightArrowPenOn);
            painter->drawLine(line);
          }
          else if(signal->rappelState == Simulator::MainSignal::RappelState::TwoLines_100)
          {
            QLineF line1(rappelRectAdj.left(), rappelRectAdj.top() + rappelRectAdj.height() * 0.25,
                         rappelRectAdj.right(), rappelRectAdj.top() + rappelRectAdj.height() * 0.25);
            QLineF line2(rappelRectAdj.left(), rappelRectAdj.top() + rappelRectAdj.height() * 0.75,
                         rappelRectAdj.right(), rappelRectAdj.top() + rappelRectAdj.height() * 0.75);
            painter->setPen(signalLightArrowPenOn);
            painter->drawLine(line1);
            painter->drawLine(line2);
          }

          // Draw border on top
          painter->setBrush(Qt::NoBrush);
          painter->setPen(signalIndicatorBorder);
          painter->drawRect(rappelRect);
        }

        if(signal->hasSquareArrowLight)
        {
          const QPointF arrowCenter(lateralDiff, - mastLength - lightDiameter / 2.0 - signalLightPen.widthF());
          lightRect.moveCenter(arrowCenter);
          painter->fillRect(lightRect, Qt::black);

          const float arrowDelta = lightDiameter * 0.3;
          const QPointF arrowDeltaPt(arrowDelta, -arrowDelta);

          painter->setPen(signal->isArrowLightOn() ? signalLightArrowPenOn : signalLightArrowPenOff);
          painter->drawLine(arrowCenter + arrowDeltaPt,
                            arrowCenter - arrowDeltaPt * 0.7);

          const QPointF bottomLeft = arrowCenter - arrowDeltaPt;
          const QPointF triangle[3] =
              {
                  bottomLeft,
                  bottomLeft + QPointF(arrowDelta / 2.0, -arrowDelta),
                  bottomLeft + QPointF(arrowDelta, -arrowDelta / 2.0)
              };

          painter->setBrush(Qt::darkGray);
          painter->drawConvexPolygon(triangle, 3);
        }

        break;
      }
      case Object::Type::AuxSignal:
      {
        auto signIt = m_stateData.auxSignals.find(obj.signalName);
        if(signIt == m_stateData.auxSignals.end())
          continue;

        // Draw with Y along mast
        painter->rotate(90.0);

        Simulator::AuxSignal *signal = signIt->second;

        // Adjust later placment
        qreal lateralDiff = obj.lateralDiff;
        if(signal->zoomLateralDiff)
          lateralDiff = obj.lateralDiff * (0.6 + 0.4 * m_signalsScaleFactor);

        painter->translate(lateralDiff, 0.0f);

        switch (signal->subType)
        {
        case Simulator::AuxSignal::SubType::LightDwarfSignal:
        {
          const qreal LightDwarfWidth = 1.5f * LightDwarfFactor;
          const qreal LightDwarfHeight = 2.0f * LightDwarfFactor;
          const qreal LightDwarfLightSz = 0.5f * LightDwarfFactor;

          // Background
          QPainterPath path;
          path.moveTo(0.0f, 0.0f);
          path.lineTo(-LightDwarfWidth / 2.0, 0.0);
          path.lineTo(-LightDwarfWidth / 2.0, -LightDwarfHeight);
          path.lineTo(0.0f, -LightDwarfHeight);
          path.lineTo(LightDwarfWidth * 0.3, -LightDwarfHeight * 0.8);
          path.lineTo(LightDwarfWidth / 2.0, -LightDwarfHeight * 0.55);
          path.lineTo(LightDwarfWidth / 2.0, 0.0);
          path.closeSubpath();

          painter->setPen(lightDwarfBorderPen);
          painter->setBrush(Qt::black);
          painter->drawPath(path);

          // Lights
          QRectF lightRect(0, 0, LightDwarfLightSz, LightDwarfLightSz);
          painter->setPen(Qt::NoPen);

          // L1
          lightRect.moveCenter(QPointF(- LightDwarfWidth / 2.0 + LightDwarfLightSz * 0.8, - LightDwarfLightSz * 1.5));
          painter->setBrush(signal->isLightOn(0) ? Qt::yellow : Qt::darkGray);
          painter->drawEllipse(lightRect);

          // L2
          lightRect.moveCenter(QPointF(+ LightDwarfWidth / 2.0 - LightDwarfLightSz * 0.8, - LightDwarfLightSz * 1.5));
          painter->setBrush(signal->isLightOn(1) ? Qt::yellow : Qt::darkGray);
          painter->drawEllipse(lightRect);

          // L3
          lightRect.moveCenter(QPointF(- LightDwarfWidth / 2.0 + LightDwarfLightSz * 0.8, - LightDwarfHeight + LightDwarfLightSz * 0.9));
          painter->setBrush(signal->isLightOn(2) ? Qt::yellow : Qt::darkGray);
          painter->drawEllipse(lightRect);

          break;
        }
        case Simulator::AuxSignal::SubType::RotatingDwarfSignal:
        {
          const QSizeF RotDwarBaseSz(1.8 * RotatingDwarfFactor, 0.8 * RotatingDwarfFactor);
          const QSizeF RotDwarMastSz(0.3 * RotatingDwarfFactor, 0.3 * RotatingDwarfFactor);
          const QSizeF RotDwarfSz(1.4 * RotatingDwarfFactor, 1.8 * RotatingDwarfFactor);
          const QSizeF RotDwarInnerSz(1.0 * RotatingDwarfFactor, 1.4 * RotatingDwarfFactor);
          const qreal RotDwarfLightSz = 0.6f * RotatingDwarfFactor;

          // Draw base
          QRectF baseRect(QPointF(), RotDwarBaseSz);
          baseRect.moveBottom(0.0f);
          baseRect.moveLeft(-baseRect.width() / 2.0);

          painter->fillRect(baseRect, Qt::black);

          QRectF mastRect(QPointF(), RotDwarMastSz);
          mastRect.moveLeft(-mastRect.width() / 2.0);
          mastRect.moveBottom(baseRect.top());
          painter->fillRect(mastRect, Qt::black);

          QRectF rotRect(QPointF(), RotDwarfSz);
          rotRect.moveLeft(-rotRect.width() / 2.0);
          rotRect.moveBottom(mastRect.top());

          QRectF rotInnerRect(QPointF(), RotDwarInnerSz);
          rotInnerRect.moveCenter(rotRect.center());

          QRectF rotLightRect(0, 0, RotDwarfLightSz, RotDwarfLightSz);
          rotLightRect.moveCenter(rotRect.center());

          // Now draw signal
          painter->setPen(rotDwarfBorderPenBlack);
          painter->setBrush(Qt::white);
          painter->drawRect(rotRect);

          if(signal->mPosition == 0)
          {
            // Draw diagonal black lines with clipping
            painter->save();
            painter->setClipRect(rotRect);
            painter->setPen(dwarfDiagPenBlack);

            QPointF start = rotRect.bottomLeft();
            QPointF end = rotRect.bottomRight();
            start.ry() += rotRect.width(); // 45 degrees diagonal
            const qreal step = dwarfDiagPenBlack.widthF() * 2.5;

            for(int i = 0; i < 9; i++)
            {
              painter->drawLine(start, end);
              start.ry() -= step;
              end.ry() -= step;
            }
            painter->restore();

            painter->setBrush(signal->isLightOn(0) ? Qt::darkMagenta : Qt::black);
            painter->drawEllipse(rotLightRect);
          }
          else if(signal->mPosition == 255)
          {
            painter->setBrush(Qt::NoBrush);
            painter->drawRect(rotInnerRect);

            painter->setBrush(signal->isLightOn(0) ? Qt::yellow : Qt::darkGray);
            painter->drawEllipse(rotLightRect);
          }
          else
          {
            const double offset = RotDwarfSz.width() * double(signal->mPosition) / 255.0;

            // Draw half and half with clipping
            painter->save();
            painter->setClipRect(rotRect);

            // Translate rect after using original for clipping
            rotRect.moveLeft(-RotDwarfSz.width() * 1.5 + offset);
            rotInnerRect.moveCenter(rotRect.center());
            rotLightRect.moveCenter(rotRect.center());

            painter->setBrush(Qt::white);
            painter->drawRect(rotRect);

            painter->setBrush(Qt::NoBrush);
            painter->drawRect(rotInnerRect);

            painter->setBrush(Qt::darkGray);
            painter->drawEllipse(rotLightRect);

            painter->restore();

            rotRect.moveLeft(-RotDwarfSz.width() * 0.5 + offset);
            rotLightRect.moveCenter(rotRect.center());

            // Draw diagonal black lines with HALF clipping
            QRectF halfClipRect = rotRect;
            halfClipRect.setRight(RotDwarfSz.width() / 2.0);

            painter->save();
            painter->setClipRect(halfClipRect);

            painter->setBrush(Qt::white);
            painter->drawRect(rotRect);

            painter->setPen(dwarfDiagPenBlack);

            QPointF start = rotRect.bottomLeft();
            QPointF end = rotRect.bottomRight();
            start.ry() += rotRect.width(); // 45 degrees diagonal
            const qreal step = dwarfDiagPenBlack.widthF() * 2.5;

            for(int i = 0; i < 9; i++)
            {
              painter->drawLine(start, end);
              start.ry() -= step;
              end.ry() -= step;
            }

            painter->setBrush(Qt::black);
            painter->drawEllipse(rotLightRect);

            painter->restore();
          }

          break;
        }
        default:
          assert(false);
          break;
        }

        break;
      }
      case Object::Type::ReverseDirection:
      {
        // Draw a triangle
        QPointF triangle[3] = {
          {-trainWidth, obj.lateralDiff - trainWidth},
          {trainWidth, obj.lateralDiff},
          {-trainWidth, obj.lateralDiff + trainWidth}
        };

        if(obj.allowedDirection == Object::AllowedDirections::Forward)
        {
          std::swap(triangle[0].rx(), triangle[1].rx());
          triangle[2].rx() = triangle[0].x();
        }

        painter->setBrush(Qt::green);
        painter->setPen(trackPen);
        painter->drawConvexPolygon(triangle, 3);
        break;
      }
      case Object::Type::RemoveTrain:
      {
        // Draw a triangle
        QPointF triangle[3] = {
          {-trainWidth, obj.lateralDiff - trainWidth},
          {trainWidth, obj.lateralDiff},
          {-trainWidth, obj.lateralDiff + trainWidth}
        };

        if(obj.allowedDirection == Object::AllowedDirections::Backwards)
        {
          std::swap(triangle[0].rx(), triangle[1].rx());
          triangle[2].rx() = triangle[0].x();
        }

        painter->setBrush(Qt::red);
        painter->setPen(trackPen);
        painter->drawConvexPolygon(triangle, 3);
        break;
      }
      case Object::Type::SpawnTrain:
      {
        // Draw a triangle
        QPointF triangle[3] = {
            {-trainWidth, obj.lateralDiff - trainWidth},
            {trainWidth * 2, obj.lateralDiff},
            {-trainWidth, obj.lateralDiff + trainWidth}
        };

        if(obj.allowedDirection == Object::AllowedDirections::Backwards)
        {
          std::swap(triangle[0].rx(), triangle[1].rx());
          triangle[2].rx() = triangle[0].x();
        }

        Simulator::Spawn::State spawnState = Simulator::Spawn::State::Inactive;
        Simulator::Spawn *spawn = m_stateData.spawns.at(obj.sensorIndex);
        if(spawn)
          spawnState = spawn->state;

        switch (spawnState)
        {
        default:
        case Simulator::Spawn::State::Inactive:
          painter->setBrush(Qt::gray);
          break;
        case Simulator::Spawn::State::Ready:
          painter->setBrush(Qt::darkGreen);
          break;
        case Simulator::Spawn::State::WaitingReset:
          painter->setBrush(Qt::darkCyan);
          break;
        }

        painter->setPen(borderPen);
        painter->drawConvexPolygon(triangle, 3);
        break;
      }
      case Object::Type::StationStopPoint:
      {
        if(!m_stateData.powerOn)
        {
          // Hide during simulation
          QRectF r;
          r.setSize(QSizeF(4, 2));
          r.moveCenter(QPointF(0, obj.lateralDiff));
          painter->fillRect(r, Qt::darkGreen);
        }
        break;
      }
      default:
        break;
      }


      painter->setTransform(trasf);
    }
  }
}

void SimulatorView::drawTrains(QPainter *painter)
{
  assert(m_simulator);

  const QTransform trasf = painter->transform();

  const float trainWidth = m_simulator->staticData.trainWidth;

  const Simulator::Train *const activeTrain = m_simulator->getTrainAt(m_trainIndex);

  QPen activeTrainPen(Qt::white, 1);
  activeTrainPen.setCosmetic(true);

  QPen stationStopTrainPen(Qt::black, 1);
  stationStopTrainPen.setCosmetic(true);

  painter->setPen(Qt::NoPen);

  for(auto it : m_stateData.vehicles)
  {
    const auto* vehicle = it.second;
    const auto& vehicleState = vehicle->state;
    const float length = vehicle->length;

    const auto center = (vehicleState.front.position + vehicleState.rear.position) / 2;
    const auto delta = vehicleState.front.position - vehicleState.rear.position;
    const float angle = atan2f(delta.y, delta.x);

    painter->translate(center.x, center.y);
    painter->rotate(qRadiansToDegrees(angle));

    const auto& color = colors[static_cast<size_t>(vehicle->color)];
    painter->setBrush(QColor(color.red * 255, color.green * 255, color.blue * 255));

    if(activeTrain && activeTrain == vehicle->activeTrain)
      painter->setPen(activeTrainPen);
    else
      painter->setPen(Qt::NoPen);

    QRectF vehicleRect(-length / 2, -trainWidth / 2,
                       length, trainWidth);
    painter->drawRect(vehicleRect);

    if(activeTrain && activeTrain->state.isOnStationStop)
    {
      QRectF adjVehicleRect = vehicleRect.adjusted(1, 1, -1, -1);
      painter->setPen(stationStopTrainPen);
      painter->setBrush(Qt::NoBrush);
      painter->drawRect(adjVehicleRect);
    }

    painter->setTransform(trasf);
  }
}

void SimulatorView::drawMisc(QPainter *painter)
{
  assert(m_simulator);

  QPen miscPen;
  miscPen.setWidth(1);
  miscPen.setCosmetic(false);

  const QTransform trasf = painter->transform();

  for(const auto& item : m_simulator->staticData.misc)
  {
    const auto& color = colors[static_cast<size_t>(item.color)];
    miscPen.setColor(QColor(color.red * 255, color.green * 255, color.blue * 255));
    painter->setPen(miscPen);

    switch(item.type)
    {
    case Simulator::Misc::Type::Rectangle:
      painter->translate(item.origin.x, item.origin.y);
      painter->rotate(qRadiansToDegrees(item.rotation));

      painter->drawRect(QRectF(0, 0, item.width, item.height));

      painter->setTransform(trasf);
      break;
    }
  }
}

bool SimulatorView::event(QEvent *e)
{
  if(m_simulator && e->type() == QEvent::ToolTip)
  {
    QHelpEvent *ev = static_cast<QHelpEvent *>(e);
    showItemTooltip(mapToSim(ev->pos()), ev);
    return true;
  }

  return QWidget::event(e);
}

void SimulatorView::keyPressEvent(QKeyEvent* e)
{
  if(!m_simulator) [[unlikely]]
  {
    return QWidget::keyPressEvent(e);
  }

    switch(e->key())
    {
    case Qt::Key_1:
    case Qt::Key_2:
    case Qt::Key_3:
    case Qt::Key_4:
    case Qt::Key_5:
    case Qt::Key_6:
    case Qt::Key_7:
    case Qt::Key_8:
    case Qt::Key_9:
    {
      const size_t trainIndex = static_cast<size_t>(e->key() - Qt::Key_1);
      std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
      Simulator::Train *train = m_simulator->getTrainAt(trainIndex);
      if(train)
      {
        m_trainIndex = trainIndex;
      }
      break;
    }
    case Qt::Key_W:
    case Qt::Key_E:
    {
      std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());

      const size_t trainCount = m_simulator->stateData().trains.size();
      size_t trainIndex = m_trainIndex;

      if(trainCount == 0 || (e->key() == Qt::Key_W && trainIndex == 0) || trainIndex == Simulator::invalidIndex)
      {
        m_trainIndex = 0;
        return;
      }


      if(e->key() == Qt::Key_W)
        trainIndex--;
      else
        trainIndex++;

      trainIndex = std::min(trainIndex, trainCount - 1);
      Simulator::Train *train = m_simulator->getTrainAt(trainIndex);
      if(train)
      {
        m_trainIndex = trainIndex;
      }
      else
      {
        m_trainIndex = 0;
      }
      break;
    }
    case Qt::Key_Up:
    {
      std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
      Simulator::Train *train = m_simulator->getTrainAt(m_trainIndex);
      if(train)
        m_simulator->applyTrainSpeedDelta(train, train->speedMax / 20);
      break;
    }
    case Qt::Key_Down:
    {
      std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
      Simulator::Train *train = m_simulator->getTrainAt(m_trainIndex);
      if(train)
        m_simulator->applyTrainSpeedDelta(train, -train->speedMax / 20);
      break;
    }
    case Qt::Key_Right:
    case Qt::Key_Left:
    {
      std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
      Simulator::Train *train = m_simulator->getTrainAt(m_trainIndex);
      if(train)
      {
        bool dir = (e->key() == Qt::Key_Left);
        if(m_simulator->isTrainDirectionInverted(train))
          dir = !dir;
        m_simulator->setTrainDirection(train, dir);
      }
      break;
    }

    case Qt::Key_Space:
    {
      std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
      Simulator::Train *train = m_simulator->getTrainAt(m_trainIndex);
      if(train)
      {
        m_simulator->setTrainMode(train, Simulator::TrainState::Mode::Manual);
        m_simulator->setTrainSpeed(train, 0.0f, true);
      }
      break;
    }
    case Qt::Key_Escape:
      m_simulator->stopAllTrains();
      break;

    case Qt::Key_Delete:
    case Qt::Key_Backspace:
      userAskRemoveTrain(m_trainIndex);
      break;

    case Qt::Key_A:
    {
      std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
      Simulator::Train *train = m_simulator->getTrainAt(m_trainIndex);
      if(train)
        m_simulator->setTrainMode(train, Simulator::TrainState::Mode::Automatic);
      break;
    }

    case Qt::Key_S:
    {
      std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
      Simulator::Train *train = m_simulator->getTrainAt(m_trainIndex);
      if(train)
        m_simulator->setTrainMode(train, Simulator::TrainState::Mode::SemiAutomatic);
      break;
    }

    case Qt::Key_M:
    {
      std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
      Simulator::Train *train = m_simulator->getTrainAt(m_trainIndex);
      if(train)
        m_simulator->setTrainMode(train, Simulator::TrainState::Mode::Manual);
      break;
    }

    case Qt::Key_B:
    {
      std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
      m_simulator->liftRestrictions(true);
      break;
    }
    case Qt::Key_N:
    {
      std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
      m_simulator->liftRestrictions(false);
      break;
    }

    case Qt::Key_P:
      m_simulator->togglePowerOn();
      break;

    default:
      return QWidget::keyPressEvent(e);
    }
}

void SimulatorView::mousePressEvent(QMouseEvent* e)
{
  if(e->button() == Qt::LeftButton)
  {
    m_leftClickMousePos = e->pos();
    resetSegmentHover();
  }
  if(e->button() == Qt::RightButton)
  {
    m_rightMousePos = e->pos();

    if(e->modifiers() != Qt::ControlModifier)
      setCursor(Qt::ClosedHandCursor);

    resetSegmentHover();
  }
}

void SimulatorView::mouseMoveEvent(QMouseEvent* e)
{
  if(e->buttons() & Qt::RightButton && e->modifiers() != Qt::ControlModifier)
  {
    m_zoomFit = false;

    const auto diff = m_rightMousePos - e->pos();

    m_cameraX += diff.x() / m_zoomLevel;
    m_cameraY += diff.y() / m_zoomLevel;

    m_rightMousePos = e->pos();
    update();
  }
  else if(e->buttons() == Qt::NoButton && m_simulator)
  {
    // Refresh hovered segment every 100 ms
    m_lastHoverPos = mapToSim(e->pos());
    if(!segmentHoverTimer.isActive())
      segmentHoverTimer.start(std::chrono::milliseconds(100), this);
  }
}

void SimulatorView::mouseReleaseEvent(QMouseEvent* e)
{
  if(e->button() == Qt::LeftButton)
  {
    auto diff = m_leftClickMousePos - e->pos();
    if(std::abs(diff.x()) <= 2 && std::abs(diff.y()) <= 2)
    {
      const bool shiftPressed = e->modifiers().testFlag(Qt::ShiftModifier);
      mouseLeftClick(mapToSim(m_leftClickMousePos), shiftPressed);
    }
  }
  if(e->button() == Qt::RightButton)
  {
    setCursor(Qt::ArrowCursor);
  }
}

void SimulatorView::wheelEvent(QWheelEvent* e)
{
  m_zoomFit = false;
  if(e->angleDelta().y() < 0)
  {
    zoomOut();
  }
  else
  {
    zoomIn();
  }
}

void SimulatorView::timerEvent(QTimerEvent *e)
{
  if(e->timerId() == turnoutBlinkTimer.timerId())
  {
    turnoutBlinkState = !turnoutBlinkState;
    update();
    return;
  }
  else if(e->timerId() == segmentHoverTimer.timerId())
  {
    size_t newHoverSegment = getSegmentAt(m_lastHoverPos, m_simulator->staticData);
    if(m_hoverSegmentIdx != newHoverSegment)
    {
      m_hoverSegmentIdx = newHoverSegment;
      if(m_hoverSegmentIdx != Simulator::invalidIndex)
      {
        const auto& segment = m_simulator->staticData.trackSegments[m_hoverSegmentIdx];
        m_hoverSensorIdx = segment.sensor.index;
      }
      else
      {
        // Reset also sensor if not hover
        m_hoverSensorIdx = Simulator::invalidIndex;
      }
    }
  }

  QWidget::timerEvent(e);
}

void SimulatorView::contextMenuEvent(QContextMenuEvent *e)
{
  if(e->modifiers() != Qt::ControlModifier)
    return; // Use control to distinguish from right click pan

  const Simulator::Point point = mapToSim(e->pos());
  const size_t idx = getSegmentAt(point, m_simulator->staticData);
  if(idx == Simulator::invalidIndex)
    return;

  QMenu *m = new QMenu(this);
  QAction *copySegData = m->addAction(tr("Copy segment data"));
  copySegData->setEnabled(!m_stateData.powerOn);
  copySegData->setVisible(!m_stateData.powerOn);

  QAction *addTrain = m->addAction(tr("Add Train"));

  const QAction *result = m->exec(e->globalPos());
  if(result == copySegData)
  {
    const auto obj = copySegmentData(idx);
    QGuiApplication::clipboard()->setText(QString::fromStdString(obj.dump(2)));
  }
  else if(result == addTrain)
  {
    showAddTrainDialog(idx, point);
  }
}

float SimulatorView::signalsScaleFactor() const
{
  return m_signalsScaleFactor;
}

void SimulatorView::setSignalsScaleFactor(float newSignalsScaleFactor)
{
  if(m_signalsScaleFactor == newSignalsScaleFactor)
    return;

  m_signalsScaleFactor = newSignalsScaleFactor;
  emit signalScaleChanged(m_signalsScaleFactor);
  update();
}

float SimulatorView::trainSpeedFactor() const
{
  return m_stateData.trainSpeedFactor;
}

void SimulatorView::setTrainSpeedFactor(float val)
{
  if(!m_simulator)
    return;

  m_simulator->setTrainSpeedFactor(val);
}

bool SimulatorView::thinTracks() const
{
  return m_thinTracks;
}

void SimulatorView::setThinTracks(bool newThinTracks)
{
  m_thinTracks = newThinTracks;
  update();
}

void SimulatorView::mouseLeftClick(const Simulator::Point &point, bool shiftPressed)
{
  for(const auto& turnout : m_turnouts)
  {
    if(isPointInTriangle(turnout.points, point))
    {
      m_simulator->toggleTurnoutState(turnout.segmentIndex, shiftPressed);
      update();
      break;
    }
  }
}

void SimulatorView::showItemTooltip(const Simulator::Point &point, QHelpEvent *ev)
{
  if (QGuiApplication::keyboardModifiers().testFlag(Qt::ShiftModifier))
  {
    // Cursor position tooltip
    QString text = tr("X: %1\n"
                      "Y: %2")
        .arg(point.x)
        .arg(point.y);
    QToolTip::showText(ev->globalPos(), text, this);
    return;
  }

  const auto &data_ = m_simulator->staticData;
  QString text;

  int ptCount = 0;
  auto addPt = [&ptCount, &text](const QString &name, const Simulator::Point &p)
  {
    ptCount++;
    QString ptText
        = tr("%1: %2; %3").arg(name.isEmpty() ? QString::number(ptCount) : name).arg(p.x).arg(p.y);
    text.append("<br>");
    text.append(ptText);
  };

  const size_t idx = getSegmentAt(point, data_);
  if (idx == Simulator::invalidIndex)
  {
    QToolTip::hideText();
    return;
  }

  const auto &segment = data_.trackSegments.at(idx);
  switch (segment.type)
  {
  case Simulator::TrackSegment::Type::Straight:
  {
    text = tr("Straight: <b>%1</b><br>").arg(QString::fromStdString(segment.m_id));
    addPt("orig", segment.points[0]);
    addPt("end", segment.points[1]);
    text.append("<br>");
    text.append(tr("rotation: %1<br>").arg(qRadiansToDegrees(segment.rotation)));
    text.append("<br>");
    text.append(tr("strai_l: %1<br>").arg(segment.straight.length));
    break;
  }
  case Simulator::TrackSegment::Type::Curve:
  {
    text = tr("Curve: <b>%1</b><br>").arg(QString::fromStdString(segment.m_id));
    addPt("orig", segment.points[0]);
    addPt("end", segment.points[1]);
    addPt("center", segment.curves[0].center);
    text.append("<br>");
    text.append(tr("rotation: %1<br>").arg(qRadiansToDegrees(segment.rotation)));
    text.append("<br>");
    text.append(tr("radius: %1<br>").arg(segment.curves[0].radius));
    text.append(tr("angle: %1<br>").arg(qRadiansToDegrees(segment.curves[0].angle)));
    text.append(tr("curve_l: %1<br>").arg(segment.curves[0].length));
    break;
  }
  case Simulator::TrackSegment::Type::Turnout:
  case Simulator::TrackSegment::Type::Turnout3Way:
  case Simulator::TrackSegment::Type::TurnoutCurved:
  {
    QLatin1String segTypeName = QLatin1String("Turnout");
    if(segment.type == Simulator::TrackSegment::Type::Turnout3Way)
      segTypeName = QLatin1String("Turnout 3 Way");
    else if(segment.type == Simulator::TrackSegment::Type::TurnoutCurved)
      segTypeName = QLatin1String("Turnout Curved");

    text = tr("%1: <b>%2</b><br>").arg(segTypeName, QString::fromStdString(segment.m_id));
    addPt("orig", segment.points[0]);

    if(segment.type == Simulator::TrackSegment::Type::Turnout3Way)
    {
      addPt("straight", segment.points[1]);
      addPt("curve 0", segment.points[2]);
      addPt("curve 1", segment.points[3]);
    }
    else if(segment.type == Simulator::TrackSegment::Type::TurnoutCurved)
    {
      addPt("curve 0", segment.points[1]);
      addPt("curve 1", segment.points[2]);
    }
    else
    {
      addPt("straight", segment.points[1]);
      addPt("curve", segment.points[2]);
    }

    text.append("<br>");
    text.append(tr("rotation: %1<br>").arg(qRadiansToDegrees(segment.rotation)));
    text.append("<br>");

    if(segment.type == Simulator::TrackSegment::Type::Turnout3Way)
    {
      text.append(tr("radius 0: %1<br>").arg(segment.curves[0].radius));
      text.append(tr("angle 0: %1<br>").arg(qRadiansToDegrees(segment.curves[0].angle)));
      text.append(tr("curve_l 0: %1<br>").arg(segment.curves[0].length));
      text.append("<br>");
      text.append(tr("radius 1: %1<br>").arg(segment.curves[1].radius));
      text.append(tr("angle 1: %1<br>").arg(qRadiansToDegrees(segment.curves[1].angle)));
      text.append(tr("curve_l 1: %1<br>").arg(segment.curves[1].length));
      text.append("<br>");
      text.append(tr("strai_l: %1<br>").arg(segment.straight.length));
    }
    else if(segment.type == Simulator::TrackSegment::Type::TurnoutCurved)
    {
      text.append(tr("radius 0: %1<br>").arg(segment.curves[0].radius));
      text.append(tr("angle 0: %1<br>").arg(qRadiansToDegrees(segment.curves[0].angle)));
      text.append(tr("curve_l 0: %1<br>").arg(segment.curves[0].length));
      text.append("<br>");
      text.append(tr("radius 1: %1<br>").arg(segment.curves[1].radius));
      text.append(tr("angle 1: %1<br>").arg(qRadiansToDegrees(segment.curves[1].angle)));
      text.append(tr("curve_l 1: %1<br>").arg(segment.curves[1].length));
    }
    else
    {
      text.append(tr("radius: %1<br>").arg(segment.curves[0].radius));
      text.append(tr("angle: %1<br>").arg(qRadiansToDegrees(segment.curves[0].angle)));
      text.append(tr("curve_l: %1<br>").arg(segment.curves[0].length));
      text.append("<br>");
      text.append(tr("strai_l: %1<br>").arg(segment.straight.length));
    }

    text.append("<br>");
    text.append(tr("turnout addr: <b>%1</b><br>"
                   "turnout chan: <b><i>%2</i></b>")
                .arg(segment.turnout.addresses[0])
                .arg(segment.turnout.channel));
    break;
  }
  case Simulator::TrackSegment::Type::SingleSlipTurnout:
  case Simulator::TrackSegment::Type::DoubleSlipTurnout:
  {
    // TODO
    text = "TODO";
    break;
  }
  }

  if (segment.hasSensor())
  {
    const auto &sensor = data_.sensors.at(segment.sensor.index);
    text.append("<br>");
    text.append(tr("sensor addr: <b>%1</b><br>"
                   "sensor channel: <b>%2</b><br>")
                .arg(sensor.address)
                .arg(sensor.channel));
  }

  QToolTip::showText(ev->globalPos(), text, this);
}

void SimulatorView::setZoomLevel(float value)
{
  m_zoomLevel = std::clamp(value, zoomLevelMin, zoomLevelMax);
  update();
}

void SimulatorView::setImageVisible(int idx, bool val)
{
  if(idx < 0 || size_t(idx) >= m_extraImages.size())
    return;

  m_extraImages[idx].visible = val;
}

nlohmann::json SimulatorView::copySegmentData(size_t segmentIdx) const
{
  if(segmentIdx == Simulator::invalidIndex)
    return {};

  const auto& segment = m_simulator->staticData.trackSegments[segmentIdx];

  nlohmann::json obj;
  obj["id"] = segment.m_id;
  obj["x"] = segment.origin().x;
  obj["y"] = segment.origin().y;
  obj["rotation"] = qRadiansToDegrees(segment.rotation);

  return obj;
}

void SimulatorView::showAddTrainDialog(size_t segmentIndex, const Simulator::Point& point)
{
  if(segmentIndex == Simulator::invalidIndex || segmentIndex >= m_simulator->staticData.trackSegments.size())
    return;

  const auto& segment = m_simulator->staticData.trackSegments.at(segmentIndex);
  const float startPos = getSegmentPosAt(segmentIndex, point, m_simulator->staticData);
  const QString segName = QString::fromStdString(segment.m_id);

  QPointer<AddTrainDialog> dlg = new AddTrainDialog(segmentIndex, startPos, segName,
                                                    mTrainsModel, this);
  dlg->exec();
  delete dlg;
}

void SimulatorView::tick()
{
  m_stateDataPrevious.powerOn = m_stateData.powerOn;
  m_stateDataPrevious.trainSpeedFactor = m_stateData.trainSpeedFactor;
  m_stateData = m_simulator->stateData();

  if(m_trainIndex >= m_stateData.trains.size())
    m_trainIndex = m_stateData.trains.size() - 1;

  emit tickActiveChanged(m_stateData.tickActive);

  if(m_stateDataPrevious.powerOn != m_stateData.powerOn)
  {
    emit powerOnChanged(m_stateData.powerOn);
  }

  if(m_stateDataPrevious.trainSpeedFactor != m_stateData.trainSpeedFactor)
  {
    emit trainSpeedFactorChanged(m_stateData.trainSpeedFactor);
  }

  update();
}

void SimulatorView::trainAddedRemoved(bool add, size_t trainIdx)
{
  std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());

  // Avoid to shift current controlled train when list is updated
  // TODO: entirely remove this trainIndex thing!!!
  if(add)
  {
    if(m_trainIndex >= trainIdx)
      m_trainIndex++;

    if(m_trainToBeRemovedIdx != Simulator::invalidIndex && m_trainToBeRemovedIdx >= trainIdx)
      m_trainToBeRemovedIdx++;
  }
  else
  {
    if(m_trainIndex == trainIdx)
      m_trainIndex = 0;
    else if(m_trainIndex > trainIdx)
      m_trainIndex--;

    if(m_trainToBeRemovedIdx != Simulator::invalidIndex && m_trainToBeRemovedIdx == trainIdx)
      m_trainToBeRemovedIdx = Simulator::invalidIndex;
    else if(m_trainToBeRemovedIdx != Simulator::invalidIndex && m_trainToBeRemovedIdx > trainIdx)
      m_trainToBeRemovedIdx--;
  }



}

void SimulatorView::userAskRemoveTrain(size_t trainIdx)
{
  {
    assert(m_trainToBeRemovedIdx == Simulator::invalidIndex);

    std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
    Simulator::Train *train = m_simulator->getTrainAt(trainIdx);
    if(!train)
      return;

    m_trainToBeRemovedIdx = trainIdx;
  }

  const int ret = QMessageBox::question(this, tr("Remove Train?"), tr("Remove Current Train?"));
  if(ret != QMessageBox::Yes && m_trainToBeRemovedIdx != Simulator::invalidAddress)
    return;

  {
    std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
    Simulator::Train *train = m_simulator->getTrainAt(m_trainToBeRemovedIdx);
    if(train)
      m_simulator->removeTrain(train->name, true);

    m_trainToBeRemovedIdx = Simulator::invalidIndex;
  }
}

void SimulatorView::doSetCurrentTrainIndex(size_t trainIndex)
{
  std::lock_guard<std::recursive_mutex> lock(m_simulator->stateMutex());
  Simulator::Train *train = m_simulator->getTrainAt(trainIndex);
  if(train)
  {
    m_trainIndex = trainIndex;
  }
}
