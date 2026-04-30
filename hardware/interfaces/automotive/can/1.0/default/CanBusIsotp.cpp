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

#include "CanBusIsotp.h"

#include "CloseHandle.h"

#include <android-base/logging.h>
#include <libnetdevice/can.h>
#include <libnetdevice/libnetdevice.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

namespace android::hardware::automotive::can::V1_0::implementation {

#define LOG_NDEBUG 0

/** Whether to log sent/received packets. */
static constexpr bool kSuperVerbose = true;

Return<Result> CanBusIsotp::send(const CanMessage& message) {
    std::lock_guard<std::mutex> lck(mIsUpGuard);
    if (!mIsUp) return Result::INTERFACE_DOWN;

    if (UNLIKELY(kSuperVerbose)) {
        LOG(VERBOSE) << "Sending " << toString(message);
    }

    if (message.payload.size() > ISOTP_DATA_LEN) {
        LOG(WARNING) << "Refuse to send ISO-TP message. Payload " << message.payload.size() << " bytes, exceeds max " << ISOTP_DATA_LEN;
        return Result::PAYLOAD_TOO_LONG;
    }

    struct canisotp_frame frame = {};
    frame.can_id = message.txId;
    if (message.isExtendedId) frame.can_id |= CAN_EFF_FLAG;
    if (message.remoteTransmissionRequest) frame.can_id |= CAN_RTR_FLAG;
    frame.len = message.payload.size();
    frame.flags |= CANFD_BRS;
    memcpy(frame.data, message.payload.data(), message.payload.size());

    auto spSocket = findSocketByTxId(message.txId);
    if (spSocket == nullptr) {
        auto info = createSocket(message.txId, message.rxId);
        if (info.socket == nullptr) {
            LOG(ERROR) << "Failed open socket";
            return Result::TRANSMISSION_FAILURE;
        }
        cacheSocket(info);
        spSocket = info.socket;
    }
    if (!spSocket->send(frame)) return Result::TRANSMISSION_FAILURE;

    return Result::OK;
}

SocketInfo CanBusIsotp::createSocket(uint32_t txId, uint32_t rxId)
{
    using namespace std::placeholders;
    CanIsotpSocket::ReadCallback rdcb = std::bind(&CanBusIsotp::onReadIsotp, this, _1, _2, _3, _4);
    CanIsotpSocket::ErrorCallback errcb = std::bind(&CanBusIsotp::onError, this, _1);
    SocketInfo info;
    info.txId = txId;
    info.rxId = rxId;
    info.socket = CanIsotpSocket::open(mIfname, rxId, txId, rdcb, errcb);
    return info;
}

std::shared_ptr<CanIsotpSocket> CanBusIsotp::findSocketByTxId(uint32_t canMsgTxId)
{
    std::lock_guard<std::mutex> lck(mSocketInfoMutex);
    for (auto& info : mSocketInfo) {
        if (info.txId == canMsgTxId) return info.socket;
    }

    LOG(WARNING) << "socket not found for: txId(" << canMsgTxId << ")";
    return nullptr;
}
std::shared_ptr<CanIsotpSocket> CanBusIsotp::findSocketByRxId(uint32_t canMsgRxId)
{
    std::lock_guard<std::mutex> lck(mSocketInfoMutex);
    for (auto& info : mSocketInfo) {
        if (info.rxId == canMsgRxId) return info.socket;
    }

    LOG(WARNING) << "socket not found for: rxId(" << canMsgRxId << ")";
    return nullptr;
}
void CanBusIsotp::cacheSocket(SocketInfo& socketInfo)
{
    if (findSocketByTxId(socketInfo.txId)) {
        //LOG(WARNING) << "socket alread exist";
        return;
    }
    if (findSocketByRxId(socketInfo.rxId)) {
        //LOG(WARNING) << "socket alread exist";
        return;
    }

    std::lock_guard<std::mutex> lck(mSocketInfoMutex);
    mSocketInfo.push_back(socketInfo);
}

Return<void> CanBusIsotp::listen(const hidl_vec<CanMessageFilter>& filter,
                            const sp<ICanMessageListener>& listenerCb, listen_cb _hidl_cb) {
    std::lock_guard<std::mutex> lck(mIsUpGuard);

    LOG(INFO) << "received ISOTP listenerCb(0x" << std::hex << listenerCb.get() << std::dec << ")...";
    if (listenerCb == nullptr) {
        _hidl_cb(Result::INVALID_ARGUMENTS, nullptr);
        return {};
    }
    if (!mIsUp) {
        _hidl_cb(Result::INTERFACE_DOWN, nullptr);
        return {};
    }

    for (auto& flt : filter) {
        if (!findSocketByRxId(flt.rxId)) {
            auto info = createSocket(flt.txId, flt.rxId);
            if (info.socket == nullptr) {
                LOG(ERROR) << "Failed open socket";
                return {};
            }
            cacheSocket(info);
        }
    }

    std::lock_guard<std::mutex> lckListeners(mMsgListenersGuard);

    sp<CloseHandle> closeHandle = new CloseHandle([this, listenerCb]() {
        std::lock_guard<std::mutex> lck(mMsgListenersGuard);
        std::erase_if(mMsgListeners, [&](const auto& e) { return e.callback == listenerCb; });
    });
    LOG(INFO) << "caching ISOTP listenerCb(0x" << std::hex << listenerCb.get() << std::dec << ")";
    mMsgListeners.emplace_back(CanMessageListener{listenerCb, filter, closeHandle});
    auto& listener = mMsgListeners.back();

    // fix message IDs to have all zeros on bits not covered by mask
    std::for_each(listener.filter.begin(), listener.filter.end(),
                  [](auto& rule) { rule.rxId &= rule.mask; });

    _hidl_cb(Result::OK, closeHandle);
    return {};
}

CanBusIsotp::CanBusIsotp() {}

CanBusIsotp::CanBusIsotp(const std::string& ifname,
    const uint32_t bitrate, const uint32_t bitsamplepoint, const uint32_t dbitrate, const uint32_t dbitsamplepoint) :
    mIfname(ifname), mBitrate(bitrate), mBitSamplePoint(bitsamplepoint), mDBitrate(dbitrate), mDBitSamplePoint(dbitsamplepoint), mTxId(0), mRxId(0) {}

CanBusIsotp::~CanBusIsotp() {
    std::lock_guard<std::mutex> lck(mIsUpGuard);
    CHECK(!mIsUp) << "Interface is still up while being destroyed";

    std::lock_guard<std::mutex> lckListeners(mMsgListenersGuard);
    CHECK(mMsgListeners.empty()) << "Listener list is not empty while interface is being destroyed";
}

void CanBusIsotp::setErrorCallback(ErrorCallback errcb) {
    CHECK(!mIsUp) << "Can't set error callback while interface is up";
    CHECK(mErrCb == nullptr) << "Error callback is already set";
    mErrCb = errcb;
    CHECK(!mIsUp) << "Can't set error callback while interface is up";
}

ICanController::Result CanBusIsotp::preUp() {
    if (!netdevice::exists(mIfname)) {
        LOG(ERROR) << "Interface " << mIfname << " doesn't exist";
        return ICanController::Result::BAD_INTERFACE_ID;
    }

    if (mBitrate == 0 || mDBitrate == 0) {
        // interface is already up and we just want to register it
        LOG(ERROR) << "Refuse to bring up CAN interface, due to invalid rate setting: bitrate()" << mBitrate << "), dbitrate(" << mDBitrate << ")";
        return ICanController::Result::OK;
    }

    if (!netdevice::down(mIfname)) {
        LOG(ERROR) << "Can't bring " << mIfname << " down (to configure it)";
        return ICanController::Result::UNKNOWN_ERROR;
    }

    if (!netdevice::can::setBitrate(mIfname, mBitrate, mBitSamplePoint, mDBitrate, mDBitSamplePoint)) {
        LOG(ERROR) << "Can't set bitrate(" << mBitrate << "), bitSample()" << mBitSamplePoint << "), dbitrate(" << mDBitrate << "), dbitSample()" << mDBitSamplePoint << ") for " << mIfname;
        return ICanController::Result::BAD_BITRATE;
    }

    return ICanController::Result::OK;
}

bool CanBusIsotp::postDown() {
    return true;
}

ICanController::Result CanBusIsotp::up() {
    std::lock_guard<std::mutex> lck(mIsUpGuard);

    if (mIsUp) {
        LOG(WARNING) << "Interface is already up";
        return ICanController::Result::INVALID_STATE;
    }

    const auto preResult = preUp();  // set bitrates
    if (preResult != ICanController::Result::OK) return preResult;

    const auto isUp = netdevice::isUp(mIfname);
    if (!isUp.has_value()) {
        // preUp() should prepare the interface (either create or make sure it's there)
        LOG(ERROR) << "Interface " << mIfname << " didn't get prepared";
        return ICanController::Result::BAD_INTERFACE_ID;
    }

    if (!*isUp && !netdevice::up(mIfname)) {
        LOG(ERROR) << "Can't bring " << mIfname << " up";
        return ICanController::Result::UNKNOWN_ERROR;
    }
    mDownAfterUse = !*isUp;

    mIsUp = true;
    return ICanController::Result::OK;
}

void CanBusIsotp::clearMsgListeners() {
    std::vector<wp<ICloseHandle>> listenersToClose;
    {
        std::lock_guard<std::mutex> lck(mMsgListenersGuard);
        std::transform(mMsgListeners.begin(), mMsgListeners.end(),
                       std::back_inserter(listenersToClose),
                       [](const auto& e) { return e.closeHandle; });
    }

    for (auto& weakListener : listenersToClose) {
        /* Between populating listenersToClose and calling close method here, some listeners might
         * have been already removed from the original mMsgListeners list (resulting in a dangling
         * weak pointer here). It's fine - we just want to clean them up. */
        auto listener = weakListener.promote();
        if (listener != nullptr) listener->close();
    }

    std::lock_guard<std::mutex> lck(mMsgListenersGuard);
    CHECK(mMsgListeners.empty()) << "Listeners list wasn't emptied";
}

void CanBusIsotp::clearErrListeners() {
    std::lock_guard<std::mutex> lck(mErrListenersGuard);
    mErrListeners.clear();
}

Return<sp<ICloseHandle>> CanBusIsotp::listenForErrors(const sp<ICanErrorListener>& listener) {
    if (listener == nullptr) {
        return new CloseHandle();
    }

    std::lock_guard<std::mutex> upLck(mIsUpGuard);
    if (!mIsUp) {
        listener->onError(ErrorEvent::INTERFACE_DOWN, true);
        return new CloseHandle();
    }

    std::lock_guard<std::mutex> errLck(mErrListenersGuard);
    mErrListeners.emplace_back(listener);

    return new CloseHandle([this, listener]() {
        std::lock_guard<std::mutex> lck(mErrListenersGuard);
        std::erase(mErrListeners, listener);
    });
}

Return<Result> CanBusIsotp::restartWithIDs(const uint32_t txid, const uint32_t rxid)
{
    if (txid == rxid) return Result::INVALID_ARGUMENTS;
    if (txid == 0 || rxid == 0) return Result::INVALID_ARGUMENTS;

    mTxId = txid; mRxId = rxid;

    LOG(DEBUG) << "restarting CAN ISO-TP interface with new CAN IDs: txid=0x" << std::hex << mTxId << ", rxid=0x" << mRxId;
    /*
#if 0
    auto ret = ICanController::Result::UNKNOWN_ERROR;
    if (down()) {  // will clear all listeners, so client must re-register them
        ret = up();
    }
#else
    mIsotpSocket.reset();
#endif
    using namespace std::placeholders;
    CanIsotpSocket::ReadCallback rdcb = std::bind(&CanBusIsotp::onReadIsotp, this, _1, _2, _3);
    CanIsotpSocket::ErrorCallback errcb = std::bind(&CanBusIsotp::onError, this, _1);
    mIsotpSocket = CanIsotpSocket::open(mIfname, mRxId, mTxId, rdcb, errcb);
    if (!mIsotpSocket) {
        if (mDownAfterUse) netdevice::down(mIfname);
        return Result::INTERFACE_DOWN;
    }
    */

    return Result::OK;
}

bool CanBusIsotp::down() {
    std::lock_guard<std::mutex> lck(mIsUpGuard);

    if (!mIsUp) {
        LOG(WARNING) << "Interface is already down";
        return false;
    }
    mIsUp = false;

    clearMsgListeners();
    clearErrListeners();

    std::lock_guard<std::mutex> mapLock(mSocketInfoMutex);
    mSocketInfo.clear();

    bool success = true;

    if (mDownAfterUse && !netdevice::down(mIfname)) {
        LOG(ERROR) << "Can't bring " << mIfname << " down";
        // don't return yet, let's try to do best-effort cleanup
        success = false;
    }

    if (!postDown()) success = false;

    return success;
}

/**
 * Helper function to determine if a flag meets the requirements of a
 * FilterFlag. See definition of FilterFlag in types.hal
 *
 * \param filterFlag FilterFlag object to match flag against
 * \param flag bool object from CanMessage object
 */
static bool satisfiesFilterFlag(FilterFlag filterFlag, bool flag) {
    if (filterFlag == FilterFlag::DONT_CARE) return true;
    if (filterFlag == FilterFlag::SET) return flag;
    if (filterFlag == FilterFlag::NOT_SET) return !flag;
    return false;
}

/**
 * Match the filter set against message id.
 *
 * For details on the filters syntax, please see CanMessageFilter at
 * the HAL definition (types.hal).
 *
 * \param filter Filter to match against
 * \param id Message id to filter
 * \return true if the message id matches the filter, false otherwise
 */
static bool match(const hidl_vec<CanMessageFilter>& filter, CanMessageId id, bool isRtr,
                  bool isExtendedId) {
    if (filter.size() == 0) return true;

    bool anyNonExcludeRulePresent = false;
    bool anyNonExcludeRuleSatisfied = false;
    for (auto& rule : filter) {
        const bool satisfied = ((id & rule.mask) == rule.rxId) &&
                               satisfiesFilterFlag(rule.rtr, isRtr) &&
                               satisfiesFilterFlag(rule.extendedFormat, isExtendedId);

        if (rule.exclude) {
            // Any exclude rule being satisfied invalidates the whole filter set.
            if (satisfied) return false;
        } else {
            anyNonExcludeRulePresent = true;
            if (satisfied) anyNonExcludeRuleSatisfied = true;
        }
    }
    return !anyNonExcludeRulePresent || anyNonExcludeRuleSatisfied;
}

void CanBusIsotp::notifyErrorListeners(ErrorEvent err, bool isFatal) {
    std::lock_guard<std::mutex> lck(mErrListenersGuard);
    for (auto& listener : mErrListeners) {
        if (!listener->onError(err, isFatal).isOk()) {
            LOG(WARNING) << "Failed to notify listener about error";
        }
    }
}

static ErrorEvent parseErrorFrame(const struct canfd_frame& frame) {
    // decode error frame (to a degree)
    if ((frame.can_id & (CAN_ERR_BUSERROR | CAN_ERR_BUSOFF)) != 0) {
        return ErrorEvent::BUS_ERROR;
    }
    if ((frame.data[1] & CAN_ERR_CRTL_TX_OVERFLOW) != 0) {
        return ErrorEvent::TX_OVERFLOW;
    }
    if ((frame.data[1] & CAN_ERR_CRTL_RX_OVERFLOW) != 0) {
        return ErrorEvent::RX_OVERFLOW;
    }
    if ((frame.data[2] & CAN_ERR_PROT_OVERLOAD) != 0) {
        return ErrorEvent::BUS_OVERLOAD;
    }
    if ((frame.can_id & CAN_ERR_PROT) != 0) {
        return ErrorEvent::MALFORMED_INPUT;
    }
    if ((frame.can_id & (CAN_ERR_CRTL | CAN_ERR_TRX | CAN_ERR_RESTARTED)) != 0) {
        // "controller restarted" constitutes a HARDWARE_ERROR imo
        return ErrorEvent::HARDWARE_ERROR;
    }
    return ErrorEvent::UNKNOWN_ERROR;
}

void CanBusIsotp::onReadIsotp(const uint32_t rxId, const uint8_t *buffer, uint32_t len, std::chrono::nanoseconds timestamp)
{
    CanMessage message = {};
    message.rxId = (rxId==0? mRxId: rxId);
    message.payload = hidl_vec<uint8_t>(buffer, buffer + len);
    message.timestamp = timestamp.count();
    message.isExtendedId = false;
    message.remoteTransmissionRequest = false;

    if (UNLIKELY(kSuperVerbose)) {
        LOG(VERBOSE) << "Got ISO-TP message " << toString(message);
    }

    std::lock_guard<std::mutex> lck(mMsgListenersGuard);
    for (auto& listener : mMsgListeners) {
        if (!match(listener.filter, message.rxId, message.remoteTransmissionRequest,
                   message.isExtendedId))
            continue;
        if (!listener.callback->onReceive(message).isOk() && !listener.failedOnce) {
            listener.failedOnce = true;
            LOG(WARNING) << "Failed to notify listener about ISO-TP message";
        }
    }
}

void CanBusIsotp::onError(int errnoVal) {
    LOG(WARNING) << "CanBusIsotp error event(" << errnoVal <<") detected";
    auto eventType = ErrorEvent::HARDWARE_ERROR;

    if (errnoVal == ENODEV || errnoVal == ENETDOWN) {
        mDownAfterUse = false;
        eventType = ErrorEvent::INTERFACE_DOWN;
    }
    notifyErrorListeners(eventType, true);

    const auto errcb = mErrCb;
    if (errcb != nullptr) errcb();
}

}  // namespace android::hardware::automotive::can::V1_0::implementation
