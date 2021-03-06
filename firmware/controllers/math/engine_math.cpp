/**
 * @file	engine_math.cpp
 * @brief
 *
 * @date Jul 13, 2013
 * @author Andrey Belomutskiy, (c) 2012-2017
 *
 * This file is part of rusEfi - see http://rusefi.com
 *
 * rusEfi is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * rusEfi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "main.h"
#include "engine_math.h"
#include "engine_configuration.h"
#include "interpolation.h"
#include "allsensors.h"
#include "io_pins.h"
#include "trigger_decoder.h"
#include "event_registry.h"
#include "efiGpio.h"
#include "fuel_math.h"
#include "advance_map.h"

EXTERN_ENGINE
;

extern EnginePins enginePins;

/**
 * @return number of milliseconds in one crank shaft revolution
 */
floatms_t getCrankshaftRevolutionTimeMs(int rpm) {
	if (rpm == 0) {
		return NAN;
	}
	return 360 * getOneDegreeTimeMs(rpm);
}

/**
 * @brief Returns engine load according to selected engine_load_mode
 *
 */
float getEngineLoadT(DECLARE_ENGINE_PARAMETER_F) {
	efiAssert(engine!=NULL, "engine 2NULL", NAN);
	efiAssert(engineConfiguration!=NULL, "engineConfiguration 2NULL", NAN);
	switch (engineConfiguration->fuelAlgorithm) {
	case LM_PLAIN_MAF:
		if (!hasMafSensor(PASS_ENGINE_PARAMETER_F)) {
			warning(CUSTOM_OBD_17, "MAF sensor needed for current fuel algorithm");
			return NAN;
		}
		return getMafT(engineConfiguration);
	case LM_SPEED_DENSITY:
		// SD engine load is used for timing lookup but not for fuel calculation
	case LM_MAP:
		return getMap();
	case LM_ALPHA_N:
		return getTPS(PASS_ENGINE_PARAMETER_F);
	case LM_REAL_MAF: {
		return getRealMaf(PASS_ENGINE_PARAMETER_F);
	}
	default:
		warning(CUSTOM_UNKNOWN_ALGORITHM, "Unexpected engine load parameter: %d", engineConfiguration->fuelAlgorithm);
		return -1;
	}
}

void setSingleCoilDwell(engine_configuration_s *engineConfiguration) {
	for (int i = 0; i < DWELL_CURVE_SIZE; i++) {
		engineConfiguration->sparkDwellBins[i] = i + 1;
		engineConfiguration->sparkDwell[i] = 4;
	}

	engineConfiguration->sparkDwellBins[5] = 10;
	engineConfiguration->sparkDwell[5] = 4;

	engineConfiguration->sparkDwellBins[6] = 4500;
	engineConfiguration->sparkDwell[6] = 4;

	engineConfiguration->sparkDwellBins[7] = 12500;
	engineConfiguration->sparkDwell[7] = 0;
}

#if EFI_ENGINE_CONTROL || defined(__DOXYGEN__)

FuelSchedule::FuelSchedule() {
	clear();
}

void FuelSchedule::clear() {
	isReady = false;
}

bool FuelSchedule::addFuelEventsForCylinder(int i  DECLARE_ENGINE_PARAMETER_S) {
	efiAssert(engine!=NULL, "engine is NULL", false);

	if (cisnan(engine->rpmCalculator.oneDegreeUs)) {
		// in order to have fuel schedule we need to have current RPM
		// wonder if this line slows engine startup?
		return false;
	}

	/**
	 * injection phase is scheduled by injection end, so we need to step the angle back
	 * for the duration of the injection
	 *
	 * todo: since this method is not invoked within trigger event handler and
	 * engineState.injectionOffset is calculated from the same utility timer should we more that logic here?
	 */
	angle_t injectionDuration = MS2US(ENGINE(fuelMs)) / ENGINE(rpmCalculator.oneDegreeUs);
	angle_t baseAngle = ENGINE(engineState.injectionOffset) - injectionDuration;

	int index;

	injection_mode_e mode = engine->getCurrentInjectionMode(PASS_ENGINE_PARAMETER_F);

	if (mode == IM_SIMULTANEOUS) {
		index = 0;
	} else if (mode == IM_SEQUENTIAL) {
		index = getCylinderId(engineConfiguration->specs.firingOrder, i) - 1;
	} else if (mode == IM_BATCH) {
		// does not look exactly right, not too consistent with IM_SEQUENTIAL
		index = i % (engineConfiguration->specs.cylindersCount / 2);
	} else {
		warning(CUSTOM_OBD_UNEXPECTED_INJECTION_MODE, "Unexpected injection mode %d", mode);
		index = 0;
	}

	bool isSimultanious = mode == IM_SIMULTANEOUS;

	assertAngleRange(baseAngle, "addFbaseAngle");

	int cylindersCount = CONFIG(specs.cylindersCount);
	if (cylindersCount < 1) {
		warning(CUSTOM_OBD_ZERO_CYLINDER_COUNT, "temp cylindersCount %d", cylindersCount);
		return false;
	}

	float angle = baseAngle
			+ i * ENGINE(engineCycle) / cylindersCount;

	InjectorOutputPin *secondOutput;
	if (mode == IM_BATCH && CONFIG(twoWireBatchInjection)) {
		/**
		 * also fire the 2nd half of the injectors so that we can implement a batch mode on individual wires
		 */
		int secondIndex = index + (CONFIG(specs.cylindersCount) / 2);
		secondOutput = &enginePins.injectors[secondIndex];
	} else {
		secondOutput = NULL;
	}

	InjectorOutputPin *output = &enginePins.injectors[index];

	if (!isSimultanious && !isPinAssigned(output)) {
		// todo: extract method for this index math
		warning(CUSTOM_OBD_INJECTION_NO_PIN_ASSIGNED, "no_pin_inj #%s", output->name);
	}

	InjectionEvent *ev = &elements[i];
	ev->ownIndex = i;
#if EFI_UNIT_TEST
	ev->engine = engine;
#endif
	fixAngle(angle, "addFuel#1");

	ev->outputs[0] = output;
	ev->outputs[1] = secondOutput;

	ev->isSimultanious = isSimultanious;

	efiAssert(TRIGGER_SHAPE(getSize()) > 0, "uninitialized TriggerShape", false);

	TRIGGER_SHAPE(findTriggerPosition(&ev->injectionStart, angle PASS_ENGINE_PARAMETER));
#if EFI_UNIT_TEST
	printf("registerInjectionEvent angle=%f trgIndex=%d inj %d\r\n", angle, ev->injectionStart.eventIndex, index);
#endif
	return true;
}

void FuelSchedule::addFuelEvents(DECLARE_ENGINE_PARAMETER_F) {
	clear();

	for (int i = 0; i < CONFIG(specs.cylindersCount); i++) {
		InjectionEvent *ev = &elements[i];
		ev->ownIndex = i;
		bool result = addFuelEventsForCylinder(i PASS_ENGINE_PARAMETER);
		if (!result)
			return;
	}
	isReady = true;
}

#endif

floatms_t getCrankingSparkDwell(int rpm DECLARE_ENGINE_PARAMETER_S) {
	if (engineConfiguration->useConstantDwellDuringCranking) {
		return engineConfiguration->ignitionDwellForCrankingMs;
	} else {
		// technically this could be implemented via interpolate2d
		float angle = engineConfiguration->crankingChargeAngle;
		return getOneDegreeTimeMs(rpm) * angle;
	}
}

/**
 * @return Spark dwell time, in milliseconds.
 */
floatms_t getSparkDwell(int rpm DECLARE_ENGINE_PARAMETER_S) {
	float dwellMs;
	if (isCrankingR(rpm)) {
		dwellMs = getCrankingSparkDwell(rpm PASS_ENGINE_PARAMETER);
	} else {
		efiAssert(!cisnan(rpm), "invalid rpm", NAN);

		dwellMs = interpolate2d(rpm, engineConfiguration->sparkDwellBins, engineConfiguration->sparkDwell, DWELL_CURVE_SIZE);
	}

	if (cisnan(dwellMs) || dwellMs < 0) {
		firmwareError(CUSTOM_ERR_DWELL_DURATION, "invalid dwell: %f at rpm=%d", dwellMs, rpm);
	}
	return dwellMs;
}

/**
 * this method is only used on initialization
 */
int TriggerShape::findAngleIndex(float target DECLARE_ENGINE_PARAMETER_S) {
	int engineCycleEventCount = engine->triggerShape.getLength();

	efiAssert(engineCycleEventCount > 0, "engineCycleEventCount", 0);

	uint32_t left = 0;
	uint32_t right = engineCycleEventCount - 1;

	/**
	 * Let's find the last trigger angle which is less or equal to the desired angle
	 * todo: extract binary search as template method?
	 */
    while (left <= right) {
        int middle = (left + right) / 2;
		angle_t eventAngle = TRIGGER_SHAPE(eventAngles[middle]);

        if (eventAngle < target) {
            left = middle + 1;
        } else if (eventAngle > target) {
            right = middle - 1;
        } else {
            // Values are equal
            return middle;             // Key found
        }
    }
    return left - 1;
}

void TriggerShape::findTriggerPosition(event_trigger_position_s *position, angle_t angleOffset DECLARE_ENGINE_PARAMETER_S) {
	// convert engine cycle angle into trigger cycle angle
	angleOffset += tdcPosition();
	fixAngle(angleOffset, "addFuel#2");

	int index = triggerIndexByAngle[(int)angleOffset];
	angle_t eventAngle = eventAngles[index];
	if (angleOffset < eventAngle) {
		warning(CUSTOM_OBD_ANGLE_CONSTRAINT_VIOLATION, "angle constraint violation in findTriggerPosition(): %f/%f", angleOffset, eventAngle);
		return;
	}

	position->eventIndex = index;
	position->eventAngle = eventAngle;
	position->angleOffset = angleOffset - eventAngle;
}

static int order_1_THEN_3_THEN_4_THEN2[] = { 1, 3, 4, 2 };
static int order_1_THEN_2_THEN_4_THEN3[] = { 1, 2, 4, 3 };
static int order_1_THEN_3_THEN_2_THEN4[] = { 1, 3, 2, 4 };

static int order_1_2_4_5_3[] = {1, 2, 4, 5, 3};

static int order_1_THEN_5_THEN_3_THEN_6_THEN_2_THEN_4[] = { 1, 5, 3, 6, 2, 4 };
static int order_1_THEN_4_THEN_2_THEN_5_THEN_3_THEN_6[] = { 1, 4, 2, 5, 3, 6 };
static int order_1_THEN_2_THEN_3_THEN_4_THEN_5_THEN_6[] = { 1, 2, 3, 4, 5, 6 };
static int order_1_6_3_2_5_4[] = {1, 6, 3, 2, 5, 4};

static int order_1_8_4_3_6_5_7_2[] = { 1, 8, 4, 3, 6, 5, 7, 2 };

static int order_1_8_7_2_6_5_4_3[] = { 1, 8, 7, 2, 6, 5, 4, 3 };
static int order_1_5_4_2_6_3_7_8[] = { 1, 5, 4, 2, 6, 3, 7, 8 };

static int order_1_2[] = {1, 2};

static int order_1_2_3[] = {1, 2, 3};

/**
 * @param index from zero to cylindersCount - 1
 * @return cylinderId from one to cylindersCount
 */
int getCylinderId(firing_order_e firingOrder, int index) {

	switch (firingOrder) {
	case FO_1:
		return 1;
// 2 cylinder
	case FO_1_2:
		return order_1_2[index];
// 3 cylinder
	case FO_1_2_3:
		return order_1_2_3[index];
// 4 cylinder
	case FO_1_3_4_2:
		return order_1_THEN_3_THEN_4_THEN2[index];
	case FO_1_2_4_3:
		return order_1_THEN_2_THEN_4_THEN3[index];
	case FO_1_3_2_4:
		return order_1_THEN_3_THEN_2_THEN4[index];
// 5 cylinder
	case FO_1_2_4_5_3:
		return order_1_2_4_5_3[index];

// 6 cylinder
	case FO_1_5_3_6_2_4:
		return order_1_THEN_5_THEN_3_THEN_6_THEN_2_THEN_4[index];
	case FO_1_4_2_5_3_6:
		return order_1_THEN_4_THEN_2_THEN_5_THEN_3_THEN_6[index];
	case FO_1_2_3_4_5_6:
		return order_1_THEN_2_THEN_3_THEN_4_THEN_5_THEN_6[index];
	case FO_1_6_3_2_5_4:
		return order_1_6_3_2_5_4[index];

// 8 cylinder
	case FO_1_8_4_3_6_5_7_2:
		return order_1_8_4_3_6_5_7_2[index];
	case FO_1_8_7_2_6_5_4_3:
		return order_1_8_7_2_6_5_4_3[index];
	case FO_1_5_4_2_6_3_7_8:
		return order_1_5_4_2_6_3_7_8[index];

	default:
		warning(CUSTOM_OBD_23, "getCylinderId not supported for %d", firingOrder);
	}
	return 1;
}

static int getIgnitionPinForIndex(int i DECLARE_ENGINE_PARAMETER_S) {
	switch (CONFIG(ignitionMode)) {
	case IM_ONE_COIL:
		return 0;
		break;
	case IM_WASTED_SPARK: {
		return i % (CONFIG(specs.cylindersCount) / 2);
	}
		break;
	case IM_INDIVIDUAL_COILS:
		return i;
		break;

	default:
		warning(CUSTOM_OBD_24, "unsupported ignitionMode %d in initializeIgnitionActions()", engineConfiguration->ignitionMode);
		return 0;
	}
}

void TriggerShape::prepareShape(DECLARE_ENGINE_PARAMETER_F) {
	int engineCycleInt = (int) getEngineCycle(CONFIG(operationMode));
	for (int angle = 0; angle < engineCycleInt; angle++) {
		int triggerShapeIndex = findAngleIndex(angle PASS_ENGINE_PARAMETER);
		if (engineConfiguration->useOnlyRisingEdgeForTrigger)
			triggerShapeIndex = triggerShapeIndex & 0xFFFFFFFE; // we need even index for front_only
		triggerIndexByAngle[angle] = triggerShapeIndex;
	}
}

#if EFI_ENGINE_CONTROL || defined(__DOXYGEN__)

/**
 * This heavy method is only invoked in case of a configuration change or initialization.
 */
void prepareOutputSignals(DECLARE_ENGINE_PARAMETER_F) {
	ENGINE(engineCycle) = getEngineCycle(CONFIG(operationMode));

	angle_t maxTimingCorrMap = -720.0f;
	angle_t maxTimingMap = -720.0f;
	for (int rpmIndex = 0;rpmIndex<IGN_RPM_COUNT;rpmIndex++) {
		for (int l = 0;l<IGN_LOAD_COUNT;l++) {
			maxTimingCorrMap = maxF(maxTimingCorrMap, config->ignitionIatCorrTable[l][rpmIndex]);
			maxTimingMap = maxF(maxTimingMap, config->ignitionTable[l][rpmIndex]);
		}
	}

#if EFI_UNIT_TEST
	floatms_t crankingDwell = getCrankingSparkDwell(CONFIG(cranking.rpm) PASS_ENGINE_PARAMETER);

	// dwell at cranking is constant angle or constant time, dwell at cranking threshold is the highest angle duration
	// lower RPM angle duration goes up
	angle_t maxDwellAngle = crankingDwell / getOneDegreeTimeMs(CONFIG(cranking.rpm));

	printf("cranking angle %f\r\n", maxDwellAngle);

	for (int i = 0;i<DWELL_CURVE_SIZE;i++) {
		int rpm = (int)engineConfiguration->sparkDwellBins[i];
		floatms_t dwell = engineConfiguration->sparkDwell[i];
		angle_t dwellAngle = dwell / getOneDegreeTimeMs(rpm);
		printf("dwell angle %f at %d\r\n", dwellAngle, rpm);
		maxDwellAngle = maxF(maxDwellAngle, dwellAngle);
	}

	angle_t maxIatAdvanceCorr = -720;
	for (int r = 0;r<IGN_RPM_COUNT;r++) {
		for (int l = 0;l<IGN_LOAD_COUNT;l++) {
			maxIatAdvanceCorr = maxF(maxIatAdvanceCorr, config->ignitionIatCorrTable[l][r]);
		}
	}

	angle_t maxAdvance = -720;
	for (int r = 0;r<IGN_RPM_COUNT;r++) {
		for (int l = 0;l<IGN_LOAD_COUNT;l++) {
			maxAdvance = maxF(maxAdvance, config->ignitionTable[l][r]);
		}
	}

	printf("max dwell angle %f/%d/%d\r\n", maxDwellAngle, (int)maxAdvance, (int)maxIatAdvanceCorr);
#endif

#if EFI_UNIT_TEST
	printf("prepareOutputSignals %d onlyEdge=%s %s\r\n", engineConfiguration->trigger.type, boolToString(engineConfiguration->useOnlyRisingEdgeForTrigger),
			getIgnition_mode_e(engineConfiguration->ignitionMode));
#endif

	for (int i = 0; i < CONFIG(specs.cylindersCount); i++) {
		ENGINE(angleExtra[i])= ENGINE(engineCycle) * i / CONFIG(specs.cylindersCount);
		ENGINE(ignitionPin[i]) = getIgnitionPinForIndex(i PASS_ENGINE_PARAMETER);
	}

	TRIGGER_SHAPE(prepareShape(PASS_ENGINE_PARAMETER_F));
}

#endif

void setFuelRpmBin(float from, float to DECLARE_ENGINE_PARAMETER_S) {
	setTableBin(config->fuelRpmBins, FUEL_RPM_COUNT, from, to);
}

void setFuelLoadBin(float from, float to DECLARE_ENGINE_PARAMETER_S) {
	setTableBin(config->fuelLoadBins, FUEL_LOAD_COUNT, from, to);
}

void setTimingRpmBin(float from, float to DECLARE_ENGINE_PARAMETER_S) {
	setRpmBin(config->ignitionRpmBins, IGN_RPM_COUNT, from, to);
}

void setTimingLoadBin(float from, float to DECLARE_ENGINE_PARAMETER_S) {
	setTableBin(config->ignitionLoadBins, IGN_LOAD_COUNT, from, to);
}

/**
 * this method sets algorithm and ignition table scale
 */
void setAlgorithm(engine_load_mode_e algo DECLARE_ENGINE_PARAMETER_S) {
	engineConfiguration->fuelAlgorithm = algo;
	if (algo == LM_ALPHA_N) {
		setTimingLoadBin(20, 120 PASS_ENGINE_PARAMETER);
	} else if (algo == LM_SPEED_DENSITY) {
		setTableBin2(config->ignitionLoadBins, IGN_LOAD_COUNT, 20, 120, 3);
		buildTimingMap(35 PASS_ENGINE_PARAMETER);
	}
}

void setInjectorLag(float value DECLARE_ENGINE_PARAMETER_S) {
	setArrayValues(engineConfiguration->injector.battLagCorr, VBAT_INJECTOR_CURVE_SIZE, value);
}
