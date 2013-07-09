/*
 * gyrodometry-cui.hpp
 *
 *  Created on: 2012/04/17
 *      Author: tyamada
 */

#ifndef ODOMETER_CUI_HPP_
#define ODOMETER_CUI_HPP_

#include "gnd-cui.hpp"

namespace Odometer {
	const gnd::cui_command cui_cmd[] = {
			{"Quit",		'Q',	"localizer shut-off"},
			{"help",		'h',	"show help"},
			{"show",		's',	"state show mode"},
			{"stand-by",	'B',	"operation stop and wait cui-command"},
			{"start",		'o',	"start operation"},
			{"debug-log",	'D',	"change debug mode on/off"},
			{"", '\0'}
	};
}
#endif /* GYRODOMETRY_CUI_HPP_ */
