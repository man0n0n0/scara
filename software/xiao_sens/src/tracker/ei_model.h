#pragma once
// Edge Impulse model. In Studio: Deployment > Arduino library > Build, then
// unzip the archive into lib/ (giving lib/<project>_inferencing/).
// If your project is not called "wormbot", change the include below.
#include <man0n0n0-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#if !defined(EI_CLASSIFIER_OBJECT_DETECTION) || EI_CLASSIFIER_OBJECT_DETECTION != 1
#error "The tracker needs an object-detection (FOMO) impulse"
#endif
