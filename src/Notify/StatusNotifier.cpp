#include "StatusNotifier.h"

StatusNotifier::Events StatusNotifier::events = {false, false, false};

void StatusNotifier::init()
{
    _toneAlarm.init();
    _ledIndicator.init();
}

void StatusNotifier::update()
{
    _toneAlarm.update();
    _ledIndicator.update();
}
