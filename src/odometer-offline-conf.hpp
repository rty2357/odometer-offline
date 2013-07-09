/*
 * gyro-odometry-conf.hpp
 *
 *  Created on: 2011/11/27
 *      Author: tyamada
 */

#ifndef ODOMETER_CONF_HPP_
#define ODOMETER_CONF_HPP_

#include <ssmtype/pws-motor.h>
#include <ssmtype/ypspur-ad.h>

#include "ssm-odometry-err.hpp"

#include "gnd-config-file.hpp"
#include "gnd-lib-error.h"


#define __SSM_NAME_ODOMETRY__	"odometry"

 // <--- constant value definition
namespace Odometer {
	static const char proc_name[] = "odometer";

	/*
	 * @brief default configuration parameter of kinematics file
	 */
	static const gnd::conf::parameter_array<char, 256> ConfIni_KFile = {
			"k-file",
			"",
			"kinematics file path"
	};

	/*
	 * @brief default configuration parameter of ssn-name
	 */
	static const gnd::conf::parameter_array<char, 32> ConfIni_SSMName = {
			"ssm-name",
			__SSM_NAME_ODOMETRY__,
			"name of shared odometer data"
	};
	/*
	 * @brief default configuration parameter of ssm-id
	 */
	static const gnd::conf::parameter<int> ConfIni_SSMID = {
			"ssm-id",
			0,
			"id of shared gyrodometry data"
	};


	/*
	 * @brief default configuration parameter of ssm-name of a/d data
	 */
	static const gnd::conf::parameter_array<char, 256> ConfIni_ADSSMLogFile = {
			"ssm-log-ad",
			"",
			"name of shared data a/d converter reading data. This parameter need only gyro odometry"
	};


	/*
	 * @brief default configuration parameter : odometry error ratio with road surface environmental map
	 */
	static const gnd::conf::parameter_array<char, 256> ConfIni_OdometryErrSSMLogFile = {
			"ssm-log-odometry-error",
			"",
			"name of shared odometry error ratio data"
	};
	/*
	 * @brief default configuration parameter of ssm-id of a/d data
	 */
	static const gnd::conf::parameter<int> ConfIni_OdometryErrID = {
			"ssm-odometry-error-id",
			-1,
			"id of shared odometry error ratio data"
	};

	/*
	 * @brief default configuration parameter of gyro a/d converter port
	 */
	static const gnd::conf::parameter<int> ConfIni_RatioPort = {
			"ratio-port",
			0,
			"a/d converter port No of ratio"
	};
	/*
	 * @brief default configuration parameter of a/d converter bits
	 */
	static const gnd::conf::parameter<int> ConfIni_ADBits = {
			"ad-bits",
			10,
			"bits of a/d converter"
	};
	/*
	 * @brief default configuration parameter of gyro sensor voltage range
	 */
	static const gnd::conf::parameter<double> ConfIni_Voltage = {
			"voltage",
			5000,				// mV
			"voltage of a/d converter [mV]"
	};
	/*
	 * @brief default configuration parameter of gyro sensor bias
	 */
	static const gnd::conf::parameter<double> ConfIni_Bias = {
			"bias",
			ConfIni_Voltage.value / 2.0,
			"bias voltage of gyro sensor [mV]"
	};
	/*
	 * @brief default configuration parameter of gyro sensor scale factor
	 */
	static const gnd::conf::parameter<double> ConfIni_ScaleFactor = {
			"scale-factor",
			20,					// mV/deg/sec
			"scale factor of gyro sensor [mV/(deg/sec)]"
	};


	/*
	 * @brief default configuration parameter of ssm-id of motor
	 */
	static const gnd::conf::parameter_array<char, 32> ConfIni_MotorSSMName = {
			"ssm-name-pws",
			SNAME_PWS_MOTOR,
			"name of shared encorder counter data"
	};
	/*
	 * @brief default configuration parameter of ssm-id of motor
	 */
	static const gnd::conf::parameter<int> ConfIni_MotorSSMID = {
			"ssm-id-pws",
			0,
			"id of shared encorder counter data"
	};

	/*
	 * @brief configuration parameter for debug mode
	 */
	static const gnd::conf::parameter<bool> ConfIni_SpurAdjust = {
			"spur-adjust",
			false,
			"adjust ypspur-coordinator's position"
	};


	/*
	 * @brief configuration parameter for debug mode
	 */
	static const gnd::conf::parameter<bool> ConfIni_Gyrodometry = {
			"gyrodometry",
			false,
			"gyrodometry mode"
	};

	/**
	 * @brief configuration error simulation
	 */
	static const gnd::conf::parameter<double> ConfIni_ErrorSimulation = {
			"error-simulation",
			0.0,
			"error-simulation (probability of generation per 1.0m)"
	};

	/**
	 * @brief configuration error simulation (error distribution)
	 */
	static const gnd::conf::parameter_array<double, 3> ConfIni_ErrorDistribution = {
			"error-distribution",
			{0.0, 0.0, 0.0},
			"error simulation (error distribution)"
	};

	/*
	 * @brief configure parameter for debug mode
	 */
	static const gnd::conf::parameter<bool> ConfIni_Debug = {
			"debug",
			false,
			"debug mode"
	};
} // <--- constant value definition



namespace Odometer {
	/*
	 * \brief particle localizer configure
	 */
	struct proc_configuration {
		proc_configuration();											///< constructor

		gnd::conf::parameter_array<char, 256>	kfile;					///< kinematics parameter file

		gnd::conf::parameter_array<char, 32>	output_ssmname;			///< output ssm-data name
		gnd::conf::parameter<int>				output_ssmid;			///< output ssm-data id

		gnd::conf::parameter_array<char, 256>	odmerr_name;			///< input ssm-data name
		gnd::conf::parameter<int>				odmerr_id;				///< input ssm-data id

		gnd::conf::parameter_array<char, 256>	ad_ssmname;				///< a/d ssm-data name
		gnd::conf::parameter<int>				ad_ssmid;				///< a/d ssm-data id
		gnd::conf::parameter<int>				ratio_port;				///< a/d port of ratio data
		gnd::conf::parameter<int>				ad_bits;				///< number of a/d converter bits
		gnd::conf::parameter<double> 			voltage;				///< a/d port voltage [mV]
		gnd::conf::parameter<double> 			bias;					///< bias of gyro sensor
		gnd::conf::parameter<double> 			scale_factor;			///< gyro sensor scale factor

		gnd::conf::parameter_array<char, 256>	motor_ssmname;			///< pws motor ssm-data name
		gnd::conf::parameter<int>				motor_ssmid;			///< pws motor ssm-data id

		gnd::conf::parameter<bool> 				spur_adjust;			///< spur adjust mode
		gnd::conf::parameter<bool> 				gyrodometry;			///< gyrodometry mode

		gnd::conf::parameter<double>				error_simulation;		///< error simulation
		gnd::conf::parameter_array<double, 3>	error_distributuion;	///< error generate

		gnd::conf::parameter<bool> 				debug;					///< debug mode
	};
	typedef struct proc_configuration configure_parameters;



	/**
	 * @brief initialize configure to default parameter
	 */
	int proc_conf_initialize(proc_configuration *conf);
	/**
	 * @brief analyze configure file
	 */
	int proc_conf_analyze(gnd::conf::configuration *fconf, proc_configuration *confp);

	proc_configuration::proc_configuration()
	{
		proc_conf_initialize(this);
	}


	/*!
	 * @brief initialize configure
	 */
	inline
	int proc_conf_initialize(proc_configuration *conf){
		gnd_assert(!conf, -1, "invalid null pointer");

		::memcpy(&conf->kfile,			&ConfIni_KFile,				sizeof(ConfIni_KFile) );

		::memcpy(&conf->output_ssmname,	&ConfIni_SSMName,			sizeof(ConfIni_SSMName) );
		::memcpy(&conf->output_ssmid,	&ConfIni_SSMID,				sizeof(ConfIni_SSMID) );

		::memcpy(&conf->odmerr_name,	&ConfIni_OdometryErrSSMLogFile,	sizeof(ConfIni_OdometryErrSSMLogFile) );
		::memcpy(&conf->odmerr_id,		&ConfIni_OdometryErrID,		sizeof(ConfIni_OdometryErrID) );

		::memcpy(&conf->ad_ssmname,		&ConfIni_ADSSMLogFile,			sizeof(ConfIni_ADSSMLogFile) );
		::memcpy(&conf->ad_ssmid,		&ConfIni_ADSSMID,			sizeof(ConfIni_ADSSMID) );
		::memcpy(&conf->ad_bits,		&ConfIni_ADBits,			sizeof(ConfIni_ADBits) );
		::memcpy(&conf->ratio_port,		&ConfIni_RatioPort,			sizeof(ConfIni_RatioPort) );
		::memcpy(&conf->voltage,		&ConfIni_Voltage,			sizeof(ConfIni_Voltage) );
		::memcpy(&conf->bias, 			&ConfIni_Bias,				sizeof(ConfIni_Bias) );
		::memcpy(&conf->scale_factor,	&ConfIni_ScaleFactor,		sizeof(ConfIni_ScaleFactor) );

		::memcpy(&conf->motor_ssmname,	&ConfIni_MotorSSMName,		sizeof(ConfIni_MotorSSMName) );
		::memcpy(&conf->motor_ssmid,	&ConfIni_MotorSSMID,		sizeof(ConfIni_MotorSSMID) );

		::memcpy(&conf->spur_adjust,	&ConfIni_SpurAdjust,		sizeof(ConfIni_SpurAdjust) );

		::memcpy(&conf->gyrodometry,	&ConfIni_Gyrodometry,		sizeof(ConfIni_Gyrodometry) );
		::memcpy(&conf->error_simulation,	&ConfIni_ErrorSimulation,		sizeof(ConfIni_ErrorSimulation) );
		::memcpy(&conf->error_distributuion,	&ConfIni_ErrorDistribution,		sizeof(ConfIni_ErrorDistribution) );

		::memcpy(&conf->debug,			&ConfIni_Debug,				sizeof(ConfIni_Debug) );

		return 0;
	}


	/**
	 * @brief get configuration parameter
	 * @param [in]  src  : configuration parameter declaration
	 * @param [out] dest : configuration parameter
	 */
	inline
	int proc_conf_get(gnd::conf::configuration *src, proc_configuration* dest) {
		gnd_assert(!src, -1, "invalid null pointer");
		gnd_assert(!dest, -1, "invalid null pointer");

		gnd::conf::get_parameter(src, &dest->kfile);

		gnd::conf::get_parameter(src, &dest->output_ssmname);
		gnd::conf::get_parameter(src, &dest->output_ssmid);

		gnd::conf::get_parameter(src, &dest->motor_ssmname);
		gnd::conf::get_parameter(src, &dest->motor_ssmid);
		gnd::conf::get_parameter(src, &dest->odmerr_name);
		gnd::conf::get_parameter(src, &dest->odmerr_id);

		gnd::conf::get_parameter(src, &dest->ad_ssmname);
		gnd::conf::get_parameter(src, &dest->ad_ssmid);
		gnd::conf::get_parameter(src, &dest->ad_bits);
		gnd::conf::get_parameter(src, &dest->ratio_port);
		gnd::conf::get_parameter(src, &dest->voltage);
		gnd::conf::get_parameter(src, &dest->bias);
		gnd::conf::get_parameter(src, &dest->scale_factor);

		gnd::conf::get_parameter(src, &dest->spur_adjust);

		gnd::conf::get_parameter(src, &dest->gyrodometry);

		gnd::conf::get_parameter(src, &dest->error_simulation);
		gnd::conf::get_parameter(src, &dest->error_distributuion);

		gnd::conf::get_parameter(src, &dest->debug);
		return 0;
	}


	/**
	 * @brief set configuration parameter
	 * @param [out] dest : configuration parameter declaration
	 * @param [int]  src : configuration parameter
	 */
	inline
	int proc_conf_set(gnd::conf::configuration *dest, proc_configuration* src) {
		gnd_assert(!src, -1, "invalid null pointer");
		gnd_assert(!dest, -1, "invalid null pointer");

		gnd::conf::set_parameter(dest, &src->kfile);

		gnd::conf::set_parameter(dest, &src->output_ssmname);
		gnd::conf::set_parameter(dest, &src->output_ssmid);

		gnd::conf::set_parameter(dest, &src->motor_ssmname);
		gnd::conf::set_parameter(dest, &src->motor_ssmid);
		gnd::conf::set_parameter(dest, &src->odmerr_name);
		gnd::conf::set_parameter(dest, &src->odmerr_id);

		gnd::conf::set_parameter(dest, &src->ad_ssmname);
		gnd::conf::set_parameter(dest, &src->ad_ssmid);
		gnd::conf::set_parameter(dest, &src->ad_bits);
		gnd::conf::set_parameter(dest, &src->ratio_port);
		gnd::conf::set_parameter(dest, &src->voltage);
		gnd::conf::set_parameter(dest, &src->bias);
		gnd::conf::set_parameter(dest, &src->scale_factor);

		gnd::conf::get_parameter(dest, &src->spur_adjust);

		gnd::conf::set_parameter(dest, &src->gyrodometry);
		gnd::conf::set_parameter(dest, &src->error_simulation);
		gnd::conf::set_parameter(dest, &src->error_distributuion);

		gnd::conf::set_parameter(dest, &src->debug);
		return 0;
	}



	/**
	 * @brief read configuration parameter file
	 * @param [in]  f    : configuration file name
	 * @param [out] dest : configuration parameter
	 */
	inline
	int proc_conf_read(const char* f, proc_configuration* dest) {
		gnd_assert(!f, -1, "invalid null pointer");
		gnd_assert(!dest, -1, "invalid null pointer");

		{ // ---> operation
			int ret;
			gnd::conf::file_stream fs;
			// configuration file read
			if( (ret = fs.read(f)) < 0 )	return ret;

			return proc_conf_get(&fs, dest);
		} // <--- operation
	}

	/**
	 * @brief write configuration parameter file
	 * @param [in]  f  : configuration file name
	 * @param [in] src : configuration parameter
	 */
	inline
	int proc_conf_write(const char* f, proc_configuration* src){
		gnd_assert(!f, -1, "invalid null pointer");
		gnd_assert(!src, -1, "invalid null pointer");

		{ // ---> operation
			int ret;
			gnd::conf::file_stream fs;
			// convert configuration declaration
			if( (ret = proc_conf_set(&fs, src)) < 0 ) return ret;

			return fs.write(f);
		} // <--- operation
	}



};

#undef __GYRODOMETRY__


#endif /* GYRO_ASSISTED_ODOMETRY_HPP_ */
