#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstdint>
#include <webots/LED.hpp>
#include <webots/Robot.hpp>

#include "TrackerTypes.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"

extern webots::Robot *_libxr_webots_robot_handle;

class WebotsFireNotify : public LibXR::Application
{
 public:
  WebotsFireNotify(LibXR::HardwareContainer &, LibXR::ApplicationManager &app)
  {
    led_ = _libxr_webots_robot_handle->getLED("fire_led");

    auto cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsFireNotify *self, LibXR::RawData &data)
        {
          auto *msg = static_cast<TrackerSend *>(data.addr_);
          self->led_->set(msg->is_fire ? 255 : 0);
        },
        this);

    fire_notify_topic_.RegisterCallback(cb);

    app.Register(*this);
  }

  void OnMonitor() override {}

 private:
  webots::LED *led_;

  LibXR::Topic fire_notify_topic_ =
      LibXR::Topic("send", sizeof(TrackerSend));
};
