/*
 * gyro-assisted-odometry-opt.hpp
 *
 *  Created on: 2011/07/28
 *      Author: tyamada
 */

#ifndef _GYRO_ASSISTED_ODOMETRY_OPT_HPP_
#define _GYRO_ASSISTED_ODOMETRY_OPT_HPP_


#include <string.h>
#include <unistd.h>
#include <getopt.h>

#include <ssmtype/spur-odometry.h>
#include <ssmtype/pws-motor.h>

#include "odometer-offline-conf.hpp"

#ifndef Odometer
#define Odometer Odometer
#endif

namespace Odometer {
	const char ConfFile[] = "odometer.conf";
	static const char ShortOpt[] = "hg:G::k:n:i:A:a:t:b:r:s:v:m:D::E::";

	static const struct option LongOpt[] = {
		{"help", 						no_argument,		0,	'h'},
		{"conf",	 					required_argument,	0,	'g'},
		{"wirte-conf", 					optional_argument,	0,	'G'},

		{ConfIni_KFile.item,			required_argument,	0,	'k'},

		{ConfIni_SSMName.item,			required_argument,	0,	'n'},
		{ConfIni_SSMID.item,			required_argument,	0,	'i'},

		{ConfIni_ADSSMLogFile.item,		required_argument,	0,	'A'},
		{ConfIni_ADSSMID.item,			required_argument,	0,	'a'},
		{ConfIni_ADBits.item,			required_argument,	0,	't'},
		{ConfIni_RatioPort.item,		required_argument,	0,	'r'},
		{ConfIni_Bias.item,				required_argument,	0,	'b'},
		{ConfIni_ScaleFactor.item,		required_argument,	0,	's'},
		{ConfIni_Voltage.item,			required_argument,	0,	'v'},

		{ConfIni_MotorSSMName.item,		required_argument,	0,	'M'},
		{ConfIni_MotorSSMID.item,		required_argument,	0,	'm'},
		{ConfIni_Debug.item,			optional_argument,	0,	'D'},
		{ConfIni_ErrorSimulation.item,	optional_argument,	0,	'E'},
		{0, 0, 0, 0}	// end of array
	};
}

namespace Odometer {

	class proc_option
	{
	public:
		static const int RetFail = -1;		///<! return value Failure
		static const int RetHelp = 1;			///<! return value Help
		static const int RetWriteConf = 2;	///<! return value Write Configure File

	// ---> declaration
	public:
		configure_parameters *conf;

		// ---> constructor
	public:
		proc_option(){ init(); }
		proc_option(configure_parameters *p) : conf(p) { init(); }
		~proc_option(){}
	private:
		void init();
	public:
		int set(configure_parameters *p){
			conf = p;
			return 0;
		}
		// <--- constructor

		// ---> operation
	public:
		bool get_option(int aArgc, char **aArgv);
		// <--- operation
	};


	inline void proc_option::init()
	{
	}

	inline bool proc_option::get_option(int aArgc, char **aArgv)
	{
		int opt;

		while(1){
			optarg = 0;
			opt = ::getopt_long(aArgc, aArgv, ShortOpt, LongOpt, 0);
			if(opt < 0)	break;

			switch(opt){
			case 'k': ::strcpy(conf->kfile.value, optarg); break;
			// read configure
			case 'g':
			{
				if( proc_conf_read(optarg, conf) < 0){
					::fprintf(stderr, " ... [\x1b[1m\x1b[31mERROR\x1b[30m\x1b[0m]: -g option, configure file syntax error\n");
					return RetFail;
				}
			} break;

			// write configure
			case 'G': {
				proc_conf_write( optarg ? optarg : ConfFile, conf);
				::fprintf(stderr, " ... output configuration file \"\x1b[4m%s\x1b[0m\"\n", optarg ? optarg : ConfFile);
			} return RetWriteConf;

			case 'n': ::strcpy( conf->output_ssmname.value, optarg);	break;
			case 'i': conf->output_ssmid.value	= ::atoi(optarg);		break;

			case 'A': ::strcpy( conf->ad_ssmname.value, optarg);		break;
			case 'a': conf->ad_ssmid.value	= ::atoi(optarg);			break;
			case 't': conf->ad_bits.value		= ::atoi(optarg);		break;
			case 'r': conf->ratio_port.value	= ::atoi(optarg);		break;
			case 'b': conf->bias.value			= ::atof(optarg);		break;
			case 's': conf->scale_factor.value  = ::atof(optarg);		break;
			case 'v': conf->voltage.value		= ::atof(optarg);		break;

			case 'M': ::strcpy(conf->motor_ssmname.value, optarg); 	break;
			case 'm': conf->motor_ssmid.value	= ::atoi(optarg); 		break;
			// debug log-mode
			case 'D': {
				if( optarg == 0 ) {
					conf->debug.value = true;
				}
				else if( ::strncmp("on", optarg, 2) == 0){
					conf->debug.value = true;
				}
				else if( ::strncmp("off", optarg, 3) == 0){
					conf->debug.value = false;
				}
				else {
					::fprintf(stderr, " ... [\x1b[1m\x1b[31mERROR\x1b[30m\x1b[0m]: -D option, configure file syntax error input argument on/off\n");
					return RetFail;
				}
			}
			break;

			// Error Simulation
			case 'E': {
				if( optarg == 0 ) {
				}
				else if( ::strncmp("on", optarg, 2) == 0){
				}
				else if( ::strncmp("off", optarg, 3) == 0){
					conf->error_simulation.value = 0.0;
					::fprintf(stderr, " ... no error simulation\n");
				}
				else {
					::fprintf(stderr, " ... [\x1b[1m\x1b[31mERROR\x1b[30m\x1b[0m]: -E option, configure file syntax error input argument on/off\n");
					return RetFail;
				}
			}
			break;

			case 'h':
			{
				int i = 0;
				fprintf(stderr, "\t\x1b[1mNAME\x1b[0m\n");
				fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m - gyro assisted odometry\n", proc_name);
				fprintf(stderr, "\n");

				fprintf(stderr, "\t\x1b[1mSYNAPSIS\x1b[0m\n");
				fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m [\x1b[4mOPTIONS\x1b[0m]\n", proc_name);
				fprintf(stderr, "\n");

				fprintf(stderr, "\t\x1b[1mDISCRIPTION\x1b[0m\n");
				fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m estimates robot position with particle filter.\n", proc_name);

				fprintf(stderr, "\n");
				fprintf(stderr, "\t\x1b[1mOPTIONS\x1b[0m\n");
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tprint help\n");
				fprintf(stderr, "\n");
				i++;

				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mFILE\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mFILE\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tread configuration file\n");
				fprintf(stderr, "\n");
				i++;

				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mFILE\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mFILE\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\twrite configuration file\n");
				fprintf(stderr, "\n");
				i++;

				// kinematics file
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mFILE\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mFILE\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tread kinematics parameter file\n");
				fprintf(stderr, "\n");
				i++;

				// ssm-name
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mName\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mName\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tset ssm-name.\n");
				fprintf(stderr, "\n");
				i++;
				// ssm-id
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mNum\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mNum\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tset ssm-id.\n");
				fprintf(stderr, "\n");
				i++;

				// a/d ssm-name
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mName\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mName\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tset A/D ssm-id.\n");
				fprintf(stderr, "\n");
				i++;
				// a/d ssm-id
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mNum\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mNum\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tset A/D ssm-id.\n");
				fprintf(stderr, "\n");
				i++;
				// a/d converte bits
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mNum\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mNum\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tset A/D converter bits resolution. default parameter is 10 bits.\n");
				fprintf(stderr, "\n");
				i++;
				// ratio data port
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mNum\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mNum\x1b[0m", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tset A/D converter port\n");
				fprintf(stderr, "\n");
				i++;
				// gyro sensor bias
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mNum\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mNum\x1b[0m", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tset bias voltage\n");
				fprintf(stderr, "\n");
				i++;
				// gyro sensor scale factor
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mNum\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mNum\x1b[0m", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tset scale-factor\n");
				fprintf(stderr, "\n");
				i++;
				// voltage
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mNum\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mNum\x1b[0m", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tset voltage\n");
				fprintf(stderr, "\n");
				i++;

				// motor data ssm-name
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mName\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mName\x1b[0m", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tset pws motor ssm-name\n");
				fprintf(stderr, "\n");
				i++;
				// motor data ssm-id
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m \x1b[4mNum\x1b[0m, \x1b[1m--%s\x1b[0m \x1b[4mNum\x1b[0m", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tset pws motor ssm-id\n");
				fprintf(stderr, "\n");
				i++;


				return RetHelp;
			}break;
			}
		}
		return 0;
	}

};


#endif /* LOCALIZER_OPT_HPP_ */
