#pragma once

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

#include "gz-to-ros-perfect-localization/perfect_localization.hpp"

// Qt
#include <QtWidgets>

// STL
#include <memory>

/**
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include "ui_panel.h"

namespace rviz_panel {
  class TFCalibrator : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit TFCalibrator(QWidget *parent = nullptr);
    ~TFCalibrator();

    /// Load and save configuration data
    virtual void load(const rviz_common::Config &config) override;
    virtual void save(rviz_common::Config config) const override;

  private Q_SLOTS:
    void on_staticTransformXSpinBox_editingFinished();
    void on_staticTransformYSpinBox_editingFinished();
    void on_staticTransformZSpinBox_editingFinished();
    void on_staticTransformRollSpinBox_editingFinished();
    void on_staticTransformPitchSpinBox_editingFinished();
    void on_staticTransformYawSpinBox_editingFinished();
    void on_parentFrameLineEdit_editingFinished();
    void on_childFrameLineEdit_editingFinished();
    void on_gazeboTopicLineEdit_editingFinished();
    void on_entityNameLineEdit_editingFinished();

  private:
    std::unique_ptr<Ui::ui_i2r_tf_calib> ui_;
    std::shared_ptr<utils::GazeboPoseToRosBridge> node_;
    std::thread spinner_thread_;

  // protected:
  };
} // rviz_panel
