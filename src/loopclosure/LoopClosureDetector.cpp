#include "kimera-vio/loopclosure/LoopClosureDetector.h"

DEFINE_bool(
    lcd_no_optimize,
    false,
    "disable RPGO optimization after a valid loop closure is detected.");

DEFINE_bool(lcd_no_detection,
            false,
            "disable detection of potential loop closures");
