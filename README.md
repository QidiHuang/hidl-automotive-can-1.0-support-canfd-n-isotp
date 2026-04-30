# hidl-automotive-can-1.0-support-canfd-n-isotp
An extension to AOSP automotive can 1.0 HIDL for supporting sending/receiving CANFD and ISOTP frames.

Extensions are done based on Android12, but since the HIDL CAN HAL doesn't change (only can/1.0 exists) in following Android versions, this piece of code also can be applied to Android13, 14, 15, 16, etc..

If you prefer to use AIDL CAN HAL, this code might be helpful as reference.

## Integrating Source
Simply to replace AOSP hardware/interfaces/automotive/can with this repository.

## Client Code Example
Using CAN/CANFD:
``` cpp
	// some necessary code for creating DeathRecipient and listeners...
	// ......
    using namespace std::chrono_literals;
    mpCanBusService = nullptr;
    uint32_t retryCount = 300u;  // 100ms * 300 = 30s
    while (mpCanBusService == nullptr && retryCount) {
        // use mCanBusName directly, as it must be initialized by init()
        mpCanBusService = ICanBus::tryGetService(mCanBusName);
        if (mpCanBusService == nullptr) {
            if (retryCount % 10 == 0) {  // print hint every 1s
                LOG_W("waiting for service %s/%s...", ICanBus::descriptor, mCanBusName.c_str());
            }
            std::this_thread::sleep_for(100ms);
        }
        --retryCount;
    }
    if (mpCanBusService == nullptr) {
        LOG_W("failed: service %s/%s not found", ICanBus::descriptor, mCanBusName.c_str());
        // do NOT return here, because ISOTP service might be available (can be registered from separate process)
    }
    if (mpCanBusService) {
        auto retRegRecipient = mpCanBusService->linkToDeath(spDeathRecipient, 0);
        LOG_I("register death recipient %s", retRegRecipient.withDefault(false)? "done":"failed");

        if (spCanMessageHandle == nullptr) {
            if (spCanMessageListener == nullptr) {
                spCanMessageListener = new CanMessageListener(*this);
            }
            CanMessageFilter canMsgFilter {
                .txId = mCanMsgIdTx,
                .rxId = mCanMsgIdRx,
                .mask = mCanMsgIdMask,
                .exclude = mIsExcludeMatch
            };
            auto retRegMsgListener = mpCanBusService->listen(
                {canMsgFilter},
                spCanMessageListener,
                [this](Result result, const android::sp<ICloseHandle>& handle) {
                    if (result != Result::OK) {
                        LOG_E("failed: message listener rejected by CAN service %s/%s",
                            ICanBus::descriptor, mCanBusName.c_str());
                        return;
                    }
                    spCanMessageHandle = handle;
                }
            );
            if (!retRegMsgListener.isOk()) {
                LOG_W("failed registering message listener to CAN service %s/%s: %s",
                    ICanBus::descriptor, mCanBusName.c_str(), retRegMsgListener.description().c_str());
            }
        }
        if (spCanErrorHandle == nullptr) {
            auto retHandle = mpCanBusService->listenForErrors(spCanErrorListener);
            spCanErrorHandle = retHandle.withDefault(nullptr);
            if (spCanErrorHandle == nullptr) {
                LOG_W("failed registering error listener to CAN service %s/%s: %s",
                    ICanBus::descriptor, mCanBusName.c_str(), retHandle.description().c_str());
            }
        }
    }
```

Using ISOTP:
``` cpp
	// some necessary code for creating DeathRecipient and listeners...
	// ......
	using namespace std::chrono_literals;
    mpCanBusIsotpService = nullptr;
    retryCount = 300u;  // 100ms * 300 = 30s
    while (mpCanBusIsotpService == nullptr && retryCount) {
        // use mCanBusName directly, as it must be initialized by init()
        mpCanBusIsotpService = ICanBusIsotp::tryGetService(mCanBusName);
        if (mpCanBusIsotpService == nullptr) {
            if (retryCount % 10 == 0) {  // print hint every 1s
                LOG_W("waiting for service %s/%s...", ICanBusIsotp::descriptor, mCanBusName.c_str());
            }
            std::this_thread::sleep_for(100ms);
        }
        --retryCount;
    }
    if (mpCanBusIsotpService == nullptr) {
        LOG_W("failed: service %s/%s not found", ICanBusIsotp::descriptor, mCanBusName.c_str());
        // do NOT return here, because CANFD service might be available (can be registered from separate process)
    }
    if (mpCanBusIsotpService) {
        auto retRegRecipientIsotp = mpCanBusIsotpService->linkToDeath(spDeathRecipient, 0);
        LOG_I("register death recipient %s", retRegRecipientIsotp.withDefault(false)? "done":"failed");

        if (spCanIsotpMessageHandle == nullptr) {
            if (spCanIsotpMessageListener == nullptr) {
                spCanIsotpMessageListener = new CanIsotpMessageListener(*this);
            }
            CanMessageFilter canMsgFilter {
                .txId = mCanMsgIdTx,
                .rxId = mCanMsgIdRx,
                .mask = mCanMsgIdMask,
                .exclude = mIsExcludeMatch
            };
            auto retRegMsgListenerIsotp = mpCanBusIsotpService->listen(
                {canMsgFilter},
                spCanIsotpMessageListener,
                [this](Result result, const android::sp<ICloseHandle>& handle) {
                    if (result != Result::OK) {
                        LOG_E("failed: message listener rejected by CAN service %s/%s",
                            ICanBusIsotp::descriptor, mCanBusName.c_str());
                        return;
                    }
                    spCanIsotpMessageHandle = handle;
                }
            );
            if (!retRegMsgListenerIsotp.isOk()) {
                LOG_W("failed registering message listener to CAN service %s/%s: %s",
                    ICanBusIsotp::descriptor, mCanBusName.c_str(), retRegMsgListenerIsotp.description().c_str());
            }
        }
        if (spCanIsotpErrorHandle == nullptr) {
            auto retHandleIsotp = mpCanBusIsotpService->listenForErrors(spCanErrorListener);
            spCanIsotpErrorHandle = retHandleIsotp.withDefault(nullptr);
            if (spCanIsotpErrorHandle == nullptr) {
                LOG_W("failed registering error listener to CAN service %s/%s: %s",
                    ICanBusIsotp::descriptor, mCanBusName.c_str(), retHandleIsotp.description().c_str());
            }
        }
    }
```

