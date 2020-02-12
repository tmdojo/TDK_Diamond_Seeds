/*
 * Copyright (c) 2015 by
 Arturo Guadalupi <a.guadalupi@arduino.cc>
 Angelo Scialabba <a.scialabba@arduino.cc>
 Claudio Indellicati <c.indellicati@arduino.cc> <bitron.it@gmail.com>
 
 * ArduinoZero Audio library for TDK Diamond Seeds.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef AUDIOZERO_TDSS_H
#define AUDIOZERO_TDSS_H

#include "Arduino.h"
#include "Print.h"

#include <SD.h>
#include <SPI.h>


class AudioZeroClass_TDSs{
public:

	AudioZeroClass_TDSs() {};
	void begin(uint32_t sampleRate);
//	void prepare(int volume); //not implemented yet
	void play(File myFile) ;
	void end();

private:
	void dacConfigure(void);
	void tcConfigure(uint32_t sampleRate);
	bool tcIsSyncing(void);
	void tcStartCounter(void);
	void tcReset(void);
	void tcEnable(void);
	void tcDisable(void);
};

extern AudioZeroClass_TDSs AudioZero_TDSs;
#endif
