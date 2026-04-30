/*
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

#pragma once

#include "CanIsotpSocket.h"

#include <android-base/unique_fd.h>
#include <android/hardware/automotive/can/1.0/ICanBusIsotp.h>
#include <android/hardware/automotive/can/1.0/ICanController.h>
#include <utils/Mutex.h>

#include <atomic>
#include <thread>

namespace android::hardware::automotive::can::V1_0::implementation {

typedef struct {
    uint32_t txId;
    uint32_t rxId;
    std::shared_ptr<CanIsotpSocket> socket;
} SocketInfo;

struct CanBusIsotp : public ICanBusIsotp {
    using ErrorCallback = std::function<void()>;

    virtual ~CanBusIsotp();

    Return<Result> send(const CanMessage& message) override;
    Return<void> listen(const hidl_vec<CanMessageFilter>& filter,
                        const sp<ICanMessageListener>& listener, listen_cb _hidl_cb) override;
    Return<sp<ICloseHandle>> listenForErrors(const sp<ICanErrorListener>& listener) override;
    Return<Result> restartWithIDs(const uint32_t txid, const uint32_t rxid) override;

    void setErrorCallback(ErrorCallback errcb);
    ICanController::Result up();
    bool down();

    /**
     * Blank constructor, since some interface types (such as SLCAN) don't get a name until after
     * being initialized.
     *
     * If using this constructor, you MUST initialize mIfname prior to the completion of preUp().
     */
    CanBusIsotp();

    CanBusIsotp(const std::string& ifname, const uint32_t bitrate, const uint32_t bitsamplepoint, const uint32_t dbitrate, const uint32_t dbitsamplepoint);

  protected:
    /**
     * Prepare the SocketCAN interface.
     *
     * After calling this method, mIfname network interface is available and ready to be brought up.
     *
     * \return OK on success, or an error state on failure. See ICanController::Result
     */
    virtual ICanController::Result preUp();

    /**
     * Cleanup after bringing the interface down.
     *
     * This is a counterpart to preUp().
     *
     * \return true upon success and false upon failure
     */
    virtual bool postDown();

    /** Network interface name. */
    std::string mIfname;

  private:
    SocketInfo createSocket(uint32_t txId, uint32_t rxId);
    std::shared_ptr<CanIsotpSocket> findSocketByTxId(uint32_t canMsgTxId);
    std::shared_ptr<CanIsotpSocket> findSocketByRxId(uint32_t canMsgTxId);
    void cacheSocket(SocketInfo& spSocket);

    struct CanMessageListener {
        sp<ICanMessageListener> callback;
        hidl_vec<CanMessageFilter> filter;
        wp<ICloseHandle> closeHandle;
        bool failedOnce = false;
    };
    void clearMsgListeners();
    void clearErrListeners();

    void notifyErrorListeners(ErrorEvent err, bool isFatal);

    void onReadIsotp(const uint32_t rxId, const uint8_t *buffer, uint32_t len, std::chrono::nanoseconds timestamp);
    void onError(int errnoVal);

    std::mutex mMsgListenersGuard;
    std::vector<CanMessageListener> mMsgListeners GUARDED_BY(mMsgListenersGuard);

    std::mutex mErrListenersGuard;
    std::vector<sp<ICanErrorListener>> mErrListeners GUARDED_BY(mErrListenersGuard);

    //std::unique_ptr<CanIsotpSocket> mIsotpSocket;
    std::vector<SocketInfo> mSocketInfo;
    std::mutex mSocketInfoMutex;
    bool mDownAfterUse;

    /**
     * Guard for up flag is required to be held for entire time when the interface is being used
     * (i.e. message being sent), because we don't want the interface to be torn down while
     * executing that operation.
     */
    std::mutex mIsUpGuard;
    bool mIsUp GUARDED_BY(mIsUpGuard) = false;

    ErrorCallback mErrCb;

    uint32_t mBitrate;
    uint32_t mBitSamplePoint;
    uint32_t mDBitrate;
    uint32_t mDBitSamplePoint;
    uint32_t mTxId;
    uint32_t mRxId;
};

}  // namespace android::hardware::automotive::can::V1_0::implementation
