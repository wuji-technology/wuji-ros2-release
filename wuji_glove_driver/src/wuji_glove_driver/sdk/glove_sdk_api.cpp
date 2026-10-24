// Copyright 2026 Wuji Technology
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "wuji_glove_driver/sdk/glove_sdk_api.hpp"

namespace wuji_glove_driver::sdk {

GloveSdkApi GloveSdkApi::production() noexcept {
  return GloveSdkApi{&wuji_init,
                     &wuji_shutdown,
                     &wuji_last_error,
                     &wuji_connect_options_default,
                     &wuji_scan,
                     &wuji_discovered_free,
                     &wuji_connect,
                     &wuji_dev_disconnect,
                     &wuji_dev_release,
                     &wuji_dev_serial_number,
                     &wuji_dev_model,
                     &wuji_dev_is_connected,
                     &wuji_glove_get_hand_side,
                     &wuji_sub_close,
                     &wuji_glove_subscribe_hand_joint_angles,
                     &wuji_glove_subscribe_emf_poses,
                     &wuji_glove_subscribe_tip_poses,
                     &wuji_glove_subscribe_hand_skeleton,
                     &wuji_glove_subscribe_imu_palm,
                     &wuji_glove_subscribe_tactile,
                     &wuji_glove_subscribe_tactile_residual,
                     &wuji_glove_subscribe_tactile_binary,
                     &wuji_glove_subscribe_tactile_zones,
                     &wuji_glove_subscribe_tactile_point_cloud};
}

}  // namespace wuji_glove_driver::sdk
