#include "gz-to-ros-perfect-localization/tf_calibrator.hpp"
#include "gz-to-ros-perfect-localization/perfect_localization.hpp"

namespace rviz_panel {
    PerfectLocalizationTFCalibrator::PerfectLocalizationTFCalibrator(QWidget *parent)
      : rviz_common::Panel{parent}
      , ui_{std::make_unique<Ui::ui_i2r_tf_calib>()}
      , node_{nullptr}
    {
      ui_->setupUi(this);
      node_ = std::make_shared<utils::GazeboPoseToRosBridge>();
      spinner_thread_ = std::thread{ [this](){ rclcpp::spin(node_); } };
    }

    PerfectLocalizationTFCalibrator::~PerfectLocalizationTFCalibrator() {
      if (spinner_thread_.joinable()) spinner_thread_.join();
    }

    void PerfectLocalizationTFCalibrator::load(const rviz_common::Config &config) {
      // Access the perfect_localization namespace
      rviz_common::Config perf_loc_config = config.mapGetChild("perf_loc");

      // Load double values with default fallbacks
      float temp_float;
      if (perf_loc_config.mapGetFloat("static_x", &temp_float)) {
        ui_->staticTransformXSpinBox->setValue(static_cast<double>(temp_float));
      }
      if (perf_loc_config.mapGetFloat("static_y", &temp_float)) {
        ui_->staticTransformYSpinBox->setValue(static_cast<double>(temp_float));
      }
      if (perf_loc_config.mapGetFloat("static_z", &temp_float)) {
        ui_->staticTransformZSpinBox->setValue(static_cast<double>(temp_float));
      }
      if (perf_loc_config.mapGetFloat("static_roll", &temp_float)) {
        ui_->staticTransformRollSpinBox->setValue(static_cast<double>(temp_float));
      }
      if (perf_loc_config.mapGetFloat("static_pitch", &temp_float)) {
        ui_->staticTransformPitchSpinBox->setValue(static_cast<double>(temp_float));
      }
      if (perf_loc_config.mapGetFloat("static_yaw", &temp_float)) {
        ui_->staticTransformYawSpinBox->setValue(static_cast<double>(temp_float));
      }
      node_->updateStaticTransform();

      // Load string values
      QString temp_string;
      if (perf_loc_config.mapGetString("parent_frame", &temp_string)) {
        ui_->parentFrameLineEdit->setText(temp_string);
      }
      if (perf_loc_config.mapGetString("child_frame", &temp_string)) {
        ui_->childFrameLineEdit->setText(temp_string);
      }
    }

    void PerfectLocalizationTFCalibrator::save(rviz_common::Config config) const {
      Panel::save(config);
      rviz_common::Config perf_loc_config = config.mapMakeChild({"perf_loc"});

      perf_loc_config.mapSetValue("static_x", ui_->staticTransformXSpinBox->value());
      perf_loc_config.mapSetValue("static_y", ui_->staticTransformYSpinBox->value());
      perf_loc_config.mapSetValue("static_z", ui_->staticTransformZSpinBox->value());
      perf_loc_config.mapSetValue("static_roll", ui_->staticTransformRollSpinBox->value());
      perf_loc_config.mapSetValue("static_pitch", ui_->staticTransformPitchSpinBox->value());
      perf_loc_config.mapSetValue("static_yaw", ui_->staticTransformYawSpinBox->value());

      // Save string values
      perf_loc_config.mapSetValue("parent_frame", ui_->parentFrameLineEdit->text());
      perf_loc_config.mapSetValue("child_frame", ui_->childFrameLineEdit->text());
    }

    void PerfectLocalizationTFCalibrator::on_staticTransformXSpinBox_editingFinished() {
      node_->setStaticTransformX(ui_->staticTransformXSpinBox->text().toDouble());
    }

    void PerfectLocalizationTFCalibrator::on_staticTransformYSpinBox_editingFinished() {
      node_->setStaticTransformY(ui_->staticTransformYSpinBox->text().toDouble());
    }

    void PerfectLocalizationTFCalibrator::on_staticTransformZSpinBox_editingFinished() {
      node_->setStaticTransformZ(ui_->staticTransformZSpinBox->text().toDouble());
    }

    void PerfectLocalizationTFCalibrator::on_staticTransformRollSpinBox_editingFinished() {
      node_->setStaticTransformRoll(ui_->staticTransformRollSpinBox->text().toDouble());
    }

    void PerfectLocalizationTFCalibrator::on_staticTransformPitchSpinBox_editingFinished() {
      node_->setStaticTransformPitch(ui_->staticTransformPitchSpinBox->text().toDouble());
    }

    void PerfectLocalizationTFCalibrator::on_staticTransformYawSpinBox_editingFinished() {
      node_->setStaticTransformYaw(ui_->staticTransformYawSpinBox->text().toDouble());
    }

    void PerfectLocalizationTFCalibrator::on_parentFrameLineEdit_editingFinished() {
      node_->setParentFrame(ui_->parentFrameLineEdit->text().toStdString());
    }

    void PerfectLocalizationTFCalibrator::on_childFrameLineEdit_editingFinished() {
      node_->setParentFrame(ui_->childFrameLineEdit->text().toStdString());
    }

    void PerfectLocalizationTFCalibrator::on_gazeboTopicLineEdit_editingFinished() {
      ;
    }

    void PerfectLocalizationTFCalibrator::on_entityNameLineEdit_editingFinished() {
      ;
    }
} // rviz_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_panel::PerfectLocalizationTFCalibrator, rviz_common::Panel)
