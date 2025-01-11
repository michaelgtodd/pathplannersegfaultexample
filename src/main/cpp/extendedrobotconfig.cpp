#include <wpi/json.h>
#include <frc/Errors.h>

#include "extendedrobotconfig.hpp"

using namespace pathplanner;

RobotConfig ExtendedRobotConfig::from_json(wpi::json input_json)
{
	wpi::json json = wpi::json::parse(input_json.dump());

	bool isHolonomic = json.at("holonomicMode").get<bool>();
	units::kilogram_t mass { json.at("robotMass").get<double>() };
	units::kilogram_square_meter_t MOI { json.at("robotMOI").get<double>() };
	units::meter_t wheelRadius { json.at("driveWheelRadius").get<double>() };
	double gearing = json.at("driveGearing").get<double>();
	units::meters_per_second_t maxDriveSpeed { json.at("maxDriveSpeed").get<
			double>() };
	double wheelCOF = json.at("wheelCOF").get<double>();
	std::string driveMotor = json.at("driveMotorType").get<std::string>();
	units::ampere_t driveCurrentLimit {
			json.at("driveCurrentLimit").get<double>() };

	int numMotors = isHolonomic ? 1 : 2;
	frc::DCMotor gearbox = ExtendedRobotConfig::getMotorFromSettingsString(driveMotor,
			numMotors).WithReduction(gearing);

	ModuleConfig moduleConfig(wheelRadius, maxDriveSpeed, wheelCOF, gearbox,
			driveCurrentLimit, numMotors);

	if (isHolonomic) {
		units::meter_t flModuleX { json.at("flModuleX").get<double>() };
		units::meter_t flModuleY { json.at("flModuleY").get<double>() };
		units::meter_t frModuleX { json.at("frModuleX").get<double>() };
		units::meter_t frModuleY { json.at("frModuleY").get<double>() };
		units::meter_t blModuleX { json.at("blModuleX").get<double>() };
		units::meter_t blModuleY { json.at("blModuleY").get<double>() };
		units::meter_t brModuleX { json.at("brModuleX").get<double>() };
		units::meter_t brModuleY { json.at("brModuleY").get<double>() };

		return RobotConfig(mass, MOI, moduleConfig,
				{ frc::Translation2d(flModuleX, flModuleY), frc::Translation2d(
						frModuleX, frModuleY), frc::Translation2d(blModuleX,
						blModuleY), frc::Translation2d(brModuleX, brModuleY) });
	} else {
		units::meter_t trackwidth { json.at("robotTrackwidth").get<double>() };

		return RobotConfig(mass, MOI, moduleConfig, trackwidth);
	}
}

frc::DCMotor ExtendedRobotConfig::getMotorFromSettingsString(std::string motorStr,
		int numMotors) {
	if (motorStr == "krakenX60") {
		return frc::DCMotor::KrakenX60(numMotors);
	} else if (motorStr == "krakenX60FOC") {
		return frc::DCMotor::KrakenX60FOC(numMotors);
	} else if (motorStr == "falcon500") {
		return frc::DCMotor::Falcon500(numMotors);
	} else if (motorStr == "falcon500FOC") {
		return frc::DCMotor::Falcon500FOC(numMotors);
	} else if (motorStr == "vortex") {
		return frc::DCMotor::NeoVortex(numMotors);
	} else if (motorStr == "NEO") {
		return frc::DCMotor::NEO(numMotors);
	} else if (motorStr == "CIM") {
		return frc::DCMotor::CIM(numMotors);
	} else if (motorStr == "miniCIM") {
		return frc::DCMotor::MiniCIM(numMotors);
	} else {
		throw std::invalid_argument("Unknown motor type string: " + motorStr);
	}
}
