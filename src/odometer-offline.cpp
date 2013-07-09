//============================================================================
// Name        : jyro-assisted-odometry.cpp
// Author      : tyamada
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <math.h>

#include <ypspur.h>

#include <ssm.h>
#include <ssm.hpp>
#include <ssm-log.hpp>
#include <ssmtype/spur-odometry.h>
#include <ssmtype/pws-motor.h>
#include <ssmtype/ypspur-ad.h>

#include "ssm-odometry-err.hpp"

#include "odometer-offline-opt.hpp"
#include "odometer-offline-cui.hpp"

#include "gnd-random.hpp"
#include "gnd-shutoff.hpp"
#include "gnd-util.h"


int read_kinematics_config(double* radius_r, double* radius_l, double* tread, double* gear, double* count_rev, const char* fname);


const char __Debug_Log__[] = "log.txt";

int main(int argc, char *argv[]) {
	SSMLog<Spur_Odometry>	ssmlog_odometry;	// gyro_odm
	SSMLog<Spur_Odometry>	ssmlog_adjust;		// spur_adjust
	SSMLog<PWSMotor>		ssmlog_mtr;		// moter ssm
	SSMLog<YP_ad>			ssmlog_ad;			// ad
	SSMLog_OdometryErr		ssmlog_odmerr;		// odometry error ratio
	struct {
		double radius_r;
		double radius_l;
		double tread;
		double gear;
		double count_rev;
	} odm_prop;
	Odometer::proc_configuration pconf;		// process configuration
	Odometer::proc_option opt(&pconf);		// process option

	FILE* fp = 0;
	gnd::cui_reader cuireader;


	{ // ---> initialize
		uint32_t phase = 1;

		// get option
		if( opt.get_option(argc, argv) ){
			return 0;
		}

		::fprintf(stderr, "========== Initialize ==========\n");
		::fprintf(stderr, " %d. allocate signal \"\x1b[4mSIGINT\x1b[0m\" to shut-off\n", phase++);
		::fprintf(stderr, " %d. get kinematics parameter\n", phase++);
		::fprintf(stderr, " %d. initialize \x1b[4mssm\x1b[0m\n", phase++);
		::fprintf(stderr, " %d. create ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.output_ssmname.value );
		if( pconf.spur_adjust.value && ::strcmp(pconf.output_ssmname.value, SNAME_ADJUST) ) {
			::fprintf(stderr, " %d. create ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, SNAME_ADJUST );
		}
		if( pconf.gyrodometry.value ) {
			::fprintf(stderr, " %d. open ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.ad_ssmname.value );
			::fprintf(stderr, " %d. open ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.motor_ssmname.value );
		}
		if(pconf.debug.value)
			::fprintf(stderr, " %d. open debug log\n", phase++);
		::fprintf(stderr, "\n");

		{ // ---> allocate SIGINT to shut-off
			::proc_shutoff_clear();
			::proc_shutoff_alloc_signal(SIGINT);
		} // <--- allocate SIGINT to shut-off



		if( !::is_proc_shutoff() ) { // ---> set initial kinematics
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => get initial kinematics parameter\n");

			{ // ---> set-zero kinematics property
				odm_prop.radius_l = 0;
				odm_prop.radius_r = 0;
				odm_prop.tread = 0;
				odm_prop.gear = 0;
				odm_prop.count_rev = 0;
			} // <--- set-zero kinematics property

			// read kinematics configure file
			if(*pconf.kfile.value != '\0'){
				::fprintf(stderr, "    load kinematics parameter file \"\x1b[4m%s\x1b[0m\"\n", pconf.kfile.value);
				read_kinematics_config(&odm_prop.radius_r, &odm_prop.radius_l, &odm_prop.tread,
						&odm_prop.gear, &odm_prop.count_rev, pconf.kfile.value);
			}

			// check kinematic parameter
			if( odm_prop.radius_r > 0 && odm_prop.radius_l > 0 && odm_prop.tread > 0 && odm_prop.gear > 0 && odm_prop.count_rev > 0){
				::fprintf(stderr, "    load lacking parameter from \x1b[4mypsypur-coordinator\x1b[0m\n");
			}
			else {
				// initialize for using ypspur-coordinator
				if( Spur_init() < 0){
					::fprintf(stderr, "  ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to connect ypspur-coordinator.\n");
					::proc_shutoff();
				}

				// get kinmeatics from ypspur-coordinator
				if( !::is_proc_shutoff() && YP_get_parameter( YP_PARAM_RADIUS_R, &odm_prop.radius_r ) < 0 ){
					::proc_shutoff();
				}
				if( !::is_proc_shutoff() && YP_get_parameter( YP_PARAM_RADIUS_L, &odm_prop.radius_l ) < 0 ){
					::proc_shutoff();
				}
				if( !::is_proc_shutoff() && YP_get_parameter( YP_PARAM_TREAD, &odm_prop.tread ) < 0 ){
					::proc_shutoff();
				}

				if( !::is_proc_shutoff() && YP_get_parameter(YP_PARAM_COUNT_REV, &odm_prop.count_rev) < 0 ){
					::proc_shutoff();
				}
				if( !::is_proc_shutoff() && YP_get_parameter(YP_PARAM_GEAR, &odm_prop.gear) < 0 ){
					::proc_shutoff();
				}
			}
			// <--- get parameter from ypspur-coordinater

			// check kinematic parameter
			if( odm_prop.radius_r <= 0 || odm_prop.radius_l <= 0 || odm_prop.tread <= 0 || odm_prop.gear <= 0 || odm_prop.count_rev <= 0){
				::fprintf(stderr, "  => \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m : Incomplete loading kinematics parameter.\n");
				::proc_shutoff();
			}
		} // <--- set initial kinematics



		// ---> initialize ssm
		if( !::is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => initialize SSM\n");
			if( !initSSM() ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: SSM is not available.\n");
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, "   ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- initialize ssm


		// ---> ssm odometry open
		if( !::is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => create ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.output_ssmname.value );
			ssmk_odometry.data.x = 0;
			ssmk_odometry.data.y = 0;
			ssmk_odometry.data.theta = 0;
			ssmk_odometry.data.v = 0;
			ssmk_odometry.data.w = 0;
			if( !ssmk_odometry.create( pconf.output_ssmname.value, pconf.output_ssmid.value, 5, 0.005 ) ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to create \"\x1b[4m%s\x1b[0m\"\n", pconf.output_ssmname.value );
				::proc_shutoff();
			}
			else {
//				ssmlog_odometry.write();
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- ssm gyro-odometry open


		if( !::is_proc_shutoff() && pconf.spur_adjust.value && ::strcmp(pconf.output_ssmname.value, SNAME_ADJUST) ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => create ssm-data \"\x1b[4m%s\x1b[0m\"\n", SNAME_ADJUST );
			ssmlog_adjust.data.x = 0;
			ssmlog_adjust.data.y = 0;
			ssmlog_adjust.data.theta = 0;
			ssmlog_adjust.data.v = 0;
			ssmlog_adjust.data.w = 0;
			if( !ssmlog_adjust.create( SNAME_ADJUST, 0, 5, 0.005 ) ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to create \"\x1b[4m%s\x1b[0m\"\n", pconf.output_ssmname.value );
				::proc_shutoff();
			}
			else {
				ssmlog_adjust.write();
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}

		}

		// ---> ssm ad conversion open
		if( !::is_proc_shutoff() && pconf.gyrodometry.value ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.ad_ssmname.value );
			if( !ssmlog_ad.openWait(pconf.ad_ssmname.value, pconf.ad_ssmid.value, 0.0, SSM_READ) ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", pconf.ad_ssmname.value );
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- ssm ad conversion open


		// ---> ssm ad conversion open
		if( !::is_proc_shutoff() && pconf.odmerr_id.value >= 0 ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.odmerr_name.value );
			if( !ssmlog_odmerr.open(pconf.odmerr_name.value) ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", pconf.odmerr_name.value );
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- ssm ad conversion open


		// ---> ssm pws-motor open
		if( !::is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open ssm-log \"\x1b[4m%s\x1b[0m\"\n", pconf.motor_ssmname.value );
			if( !ssmlog_mtr.open( pconf.motor_ssmname.value ) ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", pconf.motor_ssmname.value );
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- ssm pws-motor open

		// initialize cui
		cuireader.set_command( Odometer::cui_cmd, sizeof( Odometer::cui_cmd ) / sizeof( Odometer::cui_cmd[0] ));

		if(pconf.debug.value){
			fp = ::fopen(__Debug_Log__, "w");
			::fprintf(fp, "#[1.time] [2.x] [3.y] [4.theta] [5.v] [6.w] [7.ad] [8.bias] [9.sf]\n");
		}

	} // <--- initialize


	{ // ---> operation
		bool show_st = true;
		ssmTimeT init_time = 0;
		ssmTimeT prev_time = -1;
		double cuito = 0;
		double bias	= pconf.bias.value;
		double bias_update_ratio = 0.01;

		bool error_generate = false;

		init_time = ssmlog_mtr.time;

		if(pconf.error_simulation.value > 0.0){ // ---> error simulation initialize
			gnd::random_set_seed();
			error_generate = pconf.error_distributuion.value[0] || pconf.error_distributuion.value[1] ||  pconf.error_distributuion.value[2];
		} // <--- error simulation initialize


		// ---> while
		while( !::is_proc_shutoff() ){
			// ---> read encoder counter
			if( ssmlog_mtr.readNext() ){
				double v, w;
				double dt;
				double vol;

				if( prev_time < 0 || ssmlog_mtr.time <= prev_time){
					prev_time = ssmlog_mtr.time;
					continue;
				}
				dt = ssmlog_mtr.time - prev_time;

				{ // ---> compute odometry
					double wr = ( 2.0 * odm_prop.radius_r * M_PI * ( (double) ssmlog_mtr.data.counter2 ) ) / ( odm_prop.count_rev * odm_prop.gear );
					double wl = ( 2.0 * odm_prop.radius_l * M_PI * ( (double) ssmlog_mtr.data.counter1 ) ) / ( odm_prop.count_rev * odm_prop.gear );
					v = (wr + wl) / 2 / dt;
					w = (wr - wl) / odm_prop.tread / dt;
				} // <--- compute odometry


				// ---> compute angular velocity with gyro-sensor
				if( pconf.gyrodometry.value ){
					// read A/D
					ssmlog_ad.readTime( ssmlog_mtr.time );
					vol = ssmlog_ad.data.ad[pconf.ratio_port.value] * pconf.voltage.value / (1 << pconf.ad_bits.value);
					// in the case of just about stationary
					// compute bias
					if( ::abs(ssmlog_mtr.data.counter1) ==0 && ::abs(ssmlog_mtr.data.counter2) ==0 ){
						bias += ( vol - bias ) * bias_update_ratio;
					}
					else {
						w = -gnd_deg2ang( 1.0 / pconf.scale_factor.value ) * (vol - bias);
					}
				} // <--- compute angular velocity with gyro-sensor


				{ // ---> transration
					ssmk_odometry.data.v = v;
					ssmk_odometry.data.w = w;
					ssmk_odometry.data.x += v * dt * ::cos(ssmk_odometry.data.theta);
					ssmk_odometry.data.y += v * dt * ::sin(ssmk_odometry.data.theta);
					ssmk_odometry.data.theta += w * dt;

					// ---> error simulation
					if( error_generate && pconf.error_simulation.value * v * dt > gnd::random_uniform() ){
						gnd::matrix::fixed<3, 1> ws, error;
						gnd::matrix::fixed<3, 3> error_distribution;

						gnd::matrix::set_zero(&error_distribution);
						error_distribution[0][0] = pconf.error_distributuion.value[0] * pconf.error_distributuion.value[0];
						error_distribution[1][1] = pconf.error_distributuion.value[1] * pconf.error_distributuion.value[1];
						error_distribution[2][2] = gnd_deg2ang(pconf.error_distributuion.value[2]) * gnd_deg2ang(pconf.error_distributuion.value[2]);

						gnd::random_gaussian_mult( &error_distribution, 3, &ws, &error );

						ssmk_odometry.data.x += v * error[0][0];
						ssmk_odometry.data.y += v * error[1][0];
						ssmk_odometry.data.theta += error[2][0];
						fprintf(fp, "#error %lf %lf %lf %lf\n", ssmk_odometry.time - init_time,
								error[0][0], error[1][0], error[2][0]);
					} // <--- error simulation

					ssmk_odometry.write( ssmlog_mtr.time );
					prev_time = ssmlog_mtr.time;
					if( ssmlog_adjust.isOpen() ) {
						ssmlog_adjust.data = ssmk_odometry.data;
						ssmlog_adjust.write( ssmlog_mtr.time );
					}

					// debug log
					if(pconf.debug.value && fp){
						fprintf(fp, "%lf %lf %lf %lf %lf %lf %d %lf\n", ssmk_odometry.time - init_time,
								ssmk_odometry.data.x, ssmk_odometry.data.y, ssmk_odometry.data.theta,
								ssmk_odometry.data.v, ssmk_odometry.data.w,
								ssmlog_ad.data.ad[pconf.ratio_port.value], bias);
					}
				} // <--- transration


			} // <--- read encoder counter

			while( !::is_proc_shutoff() && ssmlog_odmerr.readNext() ){ // ---> marge odm error
				ssmk_odometry.data.x -= ssmlog_odmerr.data.dx;
				ssmk_odometry.data.y -= ssmlog_odmerr.data.dy;
				if( !pconf.gyrodometry.value ) {
					ssmk_odometry.data.theta -=  ssmlog_odmerr.data.dtheta;
				}

				ssmk_odometry.write( ssmlog_mtr.time );
				if( ssmlog_adjust.isOpen() ) {
					ssmlog_adjust.data = ssmk_odometry.data;
					ssmlog_adjust.write( ssmlog_mtr.time );
				}

				// debug log
				if(pconf.debug.value && fp){
					fprintf(fp, "%lf %lf %lf %lf %lf %lf %d %lf # fix\n", ssmk_odometry.time - init_time,
							ssmlog_odometry.data.x, ssmlog_odometry.data.y, ssmlog_odometry.data.theta,
							ssmlog_odometry.data.v, ssmlog_odometry.data.w,
							ssmlog_ad.data.ad[pconf.ratio_port.value],
							bias);
				}
			} // ---> marge odm error

			{ // ---> cui
				int cuival = 0;
				char cuiarg[512];

				::memset(cuiarg, 0, sizeof(cuiarg));
				if( cuireader.poll(&cuival, cuiarg, sizeof(cuiarg), cuito) > 0 ){
					if( show_st ){
						// if show status mode, quit show status mode
						show_st = false;
						::fprintf(stderr, "-------------------- cui mode --------------------\n");
					}
					else {
						switch(cuival) {
						// exit
						case 'Q': ::proc_shutoff();				break;
						// help
						case 'h': cuireader.show(stderr, "   ");		break;
						// show status
						case 's': show_st = true;				break;
						// stand-by mode
						case 'B': cuito = -1;					break;
						// start
						case 'o': cuito = 0;					break;
						// debug log-mode
						case 'D': {
							::fprintf(stderr, "   => debug-log mode\n");
							if( ::strncmp("on", cuiarg, 2) == 0){
								bool flg = fp ? true : false;

								if( !fp && !(fp = fopen(__Debug_Log__, "w")) ) {
									::fprintf(stderr, "    ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"", __Debug_Log__);
									pconf.debug.value = false;
								}
								else if( flg ){
									fprintf(fp, "# 1.[time] 2.[x] 3.[y] 4.[theta] 5.[v] 6.[w] 7.[wh mean] 8.[wh ratio] 9.[trd ratio] 10.[pwm1] 11.[pwm2] 12.[counter1] 13.[counter2]\n");
									pconf.debug.value = true;
									::fprintf(stderr, "    ... \x1b[1mon\x1b[0m\n");
								}
								else {
									pconf.debug.value = true;
									::fprintf(stderr, "    ... \x1b[1mon\x1b[0m\n");
								}
							}
							else if( ::strncmp("off", cuiarg, 3) == 0){
								pconf.debug.value = false;
								::fprintf(stderr, "    ... \x1b[1moff\x1b[0m\n");
							}
							else {
								::fprintf(stderr, "   ... %s\n", pconf.debug.value ? "on": "off");
								::fprintf(stderr, "   if you want to change mode, input \"on/off\"\n");
							}
						}
						break;
						case '\0':								break;
						default:
							::fprintf(stderr, "   ... \x1b[31m\x1b[1mError\x1b[0m\x1b[39m: invalid command\n");
							::fprintf(stderr, "       Please input \x1b[4mhelp\x1b[0m/\x1b[4mh\x1b[0m to show command\n");
							break;
						}
					}
					::fprintf(stderr, "  \x1b[33m\x1b[1m%s\x1b[0m\x1b[39m > ", Odometer::proc_name);
					cuireader.poll(&cuival, cuiarg, sizeof( cuiarg ), 0);
				}
			}// ---> cui


			if( show_st ){ // ---> show status
				struct timespec cur;
				static struct timespec next;
				clock_gettime(CLOCK_REALTIME, &cur);

				if( cur.tv_sec > next.tv_sec ||
						( cur.tv_sec == next.tv_sec && cur.tv_nsec > next.tv_nsec )){

					::fprintf(stderr, "\x1b[0;0H\x1b[2J");	// display clear
					::fprintf(stderr, "-------------------- \x1b[33m\x1b[1m%s\x1b[0m\x1b[39m --------------------\n", Odometer::proc_name);
					::fprintf(stderr, "    position : %.02lf[m] %.02lf[m] %.01lf[deg]\n", ssmlog_odometry.data.x, ssmlog_odometry.data.y,  gnd_ang2deg(ssmlog_odometry.data.theta) );
					::fprintf(stderr, "    velocity : v %.02lf[m/s]  w %.03lf[deg/s]\n", ssmlog_odometry.data.v, gnd_ang2deg(ssmlog_odometry.data.w) );
					::fprintf(stderr, "        mode : %s\n", pconf.gyrodometry.value ? "gyrodometry" : "odometry" );
					::fprintf(stderr, "\n");
					::fprintf(stderr, " Push \x1b[1mEnter\x1b[0m to change CUI Mode\n");
					next = cur;
					next.tv_sec++;
				}
			} // <--- show status

			// stand-by mode
			if( cuito < 0)	continue;

		} // <--- while

	} // <--- operation



	{ // ---> finalize
		::endSSM();
		::fprintf(stderr, "\n");
		::fprintf(stderr, "Finish.\x1b[49m\n");
	} // <--- finalize

	return 0;
}



int read_kinematics_config(double* radius_r, double* radius_l, double* tread,
		double* gear, double* count_rev, const char* fname)
{
	FILE* fp;
	char buf[256];
	char s, e;
	char *p;
	int l;

	// ---> initialize
	if((fp = fopen(fname, "r")) == 0){
		return -1;
	}
	// <--- initialize

	while(1){
		::memset(buf, 0, sizeof(buf));
		if(::fgets(buf, sizeof(buf), fp) == 0)	break;

		// check comment out
		for(e = 0; *(buf+e) != '\0' && *(buf+e) != '#' && *(buf+e) != '\r' && *(buf+e) != '\n'; e++ );
		*(buf+e) = '\0';
		// delete space head
		for(s = 0; s < e && ::isspace(*(buf+s)); s++);

		p = 0;
		if(      (l = ::strlen("RADIUS_R"))	&& ::strncmp(buf + s, "RADIUS_R", l) == 0)	*radius_r = ::strtod( buf + s + l, &p);
		else if( (l = ::strlen("RADIUS_L"))	&& ::strncmp(buf + s, "RADIUS_L", l) == 0)	*radius_l = ::strtod( buf + s + l, &p);
		else if( (l = ::strlen("TREAD"))	&& ::strncmp(buf + s, "TREAD", l) == 0)		*tread = ::strtod( buf + s + l, &p);
		else if( (l = ::strlen("GEAR"))		&& ::strncmp(buf + s, "GEAR", l) == 0)		*gear = ::strtod( buf + s + l, &p);
		else if( (l = ::strlen("COUNT_REV"))&& ::strncmp(buf + s, "COUNT_REV", l) == 0)	*count_rev = ::strtod( buf + s + l, &p);

		if(!p){
			// syntax error
		}
	}

	{ // ---> finalize
		fclose(fp);
	} // <--- finalize

	return 0;
}
