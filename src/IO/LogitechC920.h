#ifndef QUADROCOPTER_LOGITECHC920_H
#define QUADROCOPTER_LOGITECHC920_H

#include "iostream"

#include "ICamera.h"

class LogitechC920 : public ICamera {
public:
    explicit LogitechC920(int devicdeId);
    void DisableAutofocus() override;
    void SetFocusToInfinity() override;
    void SetResolution(int width, int height) override;
};

#endif //QUADROCOPTER_LOGITECHC920_H
