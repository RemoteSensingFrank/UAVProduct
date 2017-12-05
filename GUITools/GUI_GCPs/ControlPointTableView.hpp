// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <QDialog>
#include <QTableWidget>
#include "document.hpp"
#pragma once

namespace control_point_GUI {

/// QT Interface to edit Landmarks GCP data:
/// Allow to edit X,Y,Z Ground Control Point coordinates
/// Allow to delete GCP
class ControlPointTableView : public QDialog
{
public:
  ControlPointTableView
  (
    Landmarks   control_points,
    QWidget *parent = nullptr
  );

  /// Update control points X,Y,Z data (if valid datum is provided)
  void update_control_points(Landmarks & control_points);

  /// Delete selected control_points row(s) on Key_Delete event
  void keyReleaseEvent(QKeyEvent* event);

private:
  Landmarks control_points_;
  QTableWidget * table_;
};

} // namespace control_point_GUI
