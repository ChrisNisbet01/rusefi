/**
 * @file	engine_math.h
 *
 * @date Jul 13, 2013
 * @author Andrey Belomutskiy, (c) 2012-2017
 */

#ifndef ENGINE_MATH_H_
#define ENGINE_MATH_H_

#include "engine_configuration.h"

#include "trigger_structure.h"
#include "table_helper.h"
#include "engine.h"

void setAlgorithm(engine_load_mode_e algo DECLARE_ENGINE_PARAMETER_S);


#if EFI_ENABLE_ASSERTS
#define assertAngleRange(angle, msg) if(angle > 10000000 || angle < -10000000) { firmwareError(OBD_PCM_Processor_Fault, "angle range %s %f", msg, angle);angle = 0;}
#else
#define assertAngleRange(angle, msg) {}
#endif

void setInjectorLag(float value DECLARE_ENGINE_PARAMETER_S);


/**
 * @brief Shifts angle into the [0..720) range for four stroke and [0..360) for two stroke
 * I guess this implementation would be faster than 'angle % engineCycle'
 */
#define fixAngle(angle, msg)														\
    do \
    { \
            assertAngleRange(angle, msg);											\
            float engineCycleDurationLocalCopy = ENGINE(engineCycle);	        \
            /* todo: split this method into 'fixAngleUp' and 'fixAngleDown'*/   \
            /*       as a performance optimization?*/                           \
            while (angle < 0)                       							\
                angle += engineCycleDurationLocalCopy;   						\
                /* todo: would 'if' work as good as 'while'? */                 \
            while (angle >= engineCycleDurationLocalCopy)						\
                angle -= engineCycleDurationLocalCopy;   						\
    } while (0) 

/**
 * @return time needed to rotate crankshaft by one degree, in milliseconds.
 * @deprecated use at least Us, maybe even Nt
 */
#define getOneDegreeTimeMs(rpm) (1000.0f * 60 / 360 / (rpm))

/**
 * @return float, time needed to rotate crankshaft by one degree, in microseconds.
 * See also engine->rpmCalculator.oneDegreeUs
 */
#define getOneDegreeTimeUs(rpm) (1000000.0f * 60 / 360 / (rpm))

/**
 * @return float, time needed to rotate crankshaft by one degree, in native clicks.
 */
#define getOneDegreeTimeNt(rpm) (US2NT(1000000) * 60.0f / 360 / (rpm))

floatms_t getCrankshaftRevolutionTimeMs(int rpm);

#define isCrankingR(rpm) ((rpm) > 0 && (rpm) < CONFIG(cranking.rpm))

float getEngineLoadT(DECLARE_ENGINE_PARAMETER_F);

floatms_t getSparkDwell(int rpm DECLARE_ENGINE_PARAMETER_S);

int getCylinderId(firing_order_e firingOrder, int index);

void setFuelRpmBin(float from, float to DECLARE_ENGINE_PARAMETER_S);
void setFuelLoadBin(float from, float to DECLARE_ENGINE_PARAMETER_S);
void setTimingRpmBin(float from, float to DECLARE_ENGINE_PARAMETER_S);
void setTimingLoadBin(float from, float to DECLARE_ENGINE_PARAMETER_S);

void setSingleCoilDwell(engine_configuration_s *engineConfiguration);

#define tdcPosition() \
		(ENGINE(triggerShape.tdcPosition) + CONFIG(globalTriggerAngleOffset))

#endif /* ENGINE_MATH_H_ */
