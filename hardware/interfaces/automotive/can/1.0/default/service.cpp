/*
 * Copyright (C) 2019 The Android Open Source Project
 * Copyright (C) 2026 Qidi Huang (huang_qi_di@hotmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "CanController.h"

#include <android-base/logging.h>
#include <hidl/HidlTransportSupport.h>
#include <libnetdevice/libnetdevice.h>

#include <canhalctrl.h>

namespace android::hardware::automotive::can::V1_0::implementation {

static void registerICanBusService()
{
    char *argv[] = {
        const_cast<char*>("up"),
        const_cast<char*>("can0"),
        const_cast<char*>("socketcan"),
        const_cast<char*>("can0"),
        const_cast<char*>("500000"),  // bitrate
        const_cast<char*>("800"),  // b-samplepoint, 800 for 80%, 750 for 75%
        const_cast<char*>("2000000"),  // dbitrate
        const_cast<char*>("800"),  // db-samplepoint, 800 for 80%, 770 for 77%
    };
    int argc = 8;
    int retCanHalCtrlStarted = start_canhalctrl(argc, argv);
    LOG(INFO) << "CAN bus status report: CanHalCtrlStarted(" << (retCanHalCtrlStarted==0?"OK":"FAIL") << ")";
}

static void canControllerService() {
    base::SetDefaultTag("CanController");
    base::SetMinimumLogSeverity(android::base::VERBOSE);
    configureRpcThreadpool(16, true);
    LOG(DEBUG) << "CAN controller service starting...";

    netdevice::useSocketDomain(AF_CAN);

    sp<CanController> canController(new CanController);
    if (canController->registerAsService("socketcan") != OK) {
        LOG(FATAL) << "Failed to register CAN controller";
        return;
    }
    LOG(INFO) << "CAN controller service ready";

    std::this_thread::sleep_for(std::chrono::seconds(1));
    registerICanBusService();

    joinRpcThreadpool();
}

}  // namespace android::hardware::automotive::can::V1_0::implementation

int main() {
    ::android::hardware::automotive::can::V1_0::implementation::canControllerService();
    return 1;  // canBusService (joinRpcThreadpool) shouldn't exit
}
