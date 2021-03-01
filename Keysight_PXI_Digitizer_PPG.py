#!/usr/bin/env python
import os
import sys
import logging
from datetime import datetime

sys.path.append('C:\Program Files (x86)\Keysight\SD1\Libraries\Python')
from BaseDriver import LabberDriver, Error, IdError
import keysightSD1

import numpy as np


class Driver(LabberDriver):
    """ This class implements the Keysight PXI digitizer for PPG use"""

    def performOpen(self, options={}):
        """Perform the operation of opening the instrument connection"""
        self.initLogger()
        # set time step and resolution
        self.nBit = 16
        self.bitRange = float(2**(self.nBit-1)-1)
        # timeout
        self.timeout_ms = int(1000 * self.dComCfg['Timeout'])
        # get PXI chassis
        self.chassis = int(self.dComCfg.get('PXI chassis', 1))
        # create AWG instance
        self.dig = keysightSD1.SD_AIN()
        AWGPart = self.dig.getProductNameBySlot(
            self.chassis, int(self.comCfg.address))
        self.log('Serial:', self.dig.getSerialNumberBySlot(
            self.chassis, int(self.comCfg.address)))
        if not isinstance(AWGPart, str):
            raise Error('Unit not available')
        # check that model is supported
        dOptionCfg = self.dInstrCfg['options']
        for validId, validName in zip(dOptionCfg['model_id'], dOptionCfg['model_str']):
            if AWGPart.find(validId)>=0:
                # id found, stop searching
                break
        else:
            # loop fell through, raise ID error
            raise IdError(AWGPart, dOptionCfg['model_id'])
        # set model
        self.setModel(validName)
        # sampling rate and number of channles is set by model
        if validName in ('M3102', 'M3302'):
            # 500 MHz models
            self.dt = 2E-9
            self.nCh = 4
        else:
            # assume 100 MHz for all other models
            self.dt = 10E-9
            self.nCh = 4
        # create list of sampled data
        self.lTrace = [np.array([])] * self.nCh
        self.dig.openWithSlot(AWGPart, self.chassis, int(self.comCfg.address))
        # get hardware version - changes numbering of channels
        hw_version = self.dig.getHardwareVersion()
        if hw_version >= 4:
            # KEYSIGHT - channel numbers start with 1
            self.ch_index_zero = 1
        else:
            # SIGNADYNE - channel numbers start with 0
            self.ch_index_zero = 0
        self.log('HW:', hw_version)


    def initLogger(self):
        ## Dir and file setup
        log_dir = os.path.expanduser("~/driver_logs/")
        log_file = "Digitizer_PPG_{:%y%m%d_%H%M%S}.log".format(datetime.now())
        log_path = os.path.join(log_dir, log_file)
        ## Create log dir if it does not exist
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        ## logger object config and init
        logging.basicConfig(filename=log_path, filemode="a",
                            level=logging.DEBUG,
                            format="%(asctime)s.%(msecs)06f %(name)-8s: %(message)s",
                            datefmt="%y-%m-%d %H:%M:%S")
        self._logger = logging.getLogger("Digitizer_PPG")
        self._logger.info("Logging initialized to {}".format(log_path))



    def getHwCh(self, n):
        """Get hardware channel number for channel n. n starts at 0"""
        return n + self.ch_index_zero


    def performClose(self, bError=False, options={}):
        """Perform the close instrument connection operation"""
        # do not check for error if close was called with an error
        try:
            # flush all memory
            for n in range(self.nCh):
                self.log('Close ch:', n, self.dig.DAQflush(self.getHwCh(n)))
            # close instrument
            self.dig.close()
        except:
            # never return error here
            pass


    def performSetValue(self, quant, value, sweepRate=0.0, options={}):
        """Perform the Set Value instrument operation. This function should
        return the actual value set by the instrument"""
        # start with setting local quant value
        quant.setValue(value)
        # check if channel-specific, if so get channel + name
        if quant.name.startswith('Ch') and len(quant.name)>6:
            ch = int(quant.name[2]) - 1
            name = quant.name[6:]
        else:
            ch, name = None, ''
        # proceed depending on command
        if quant.name in ('External Trig Source', 'External Trig Config',
                          'Trig Sync Mode'):
            extSource = int(self.getCmdStringFromValue('External Trig Source'))
            trigBehavior = int(self.getCmdStringFromValue('External Trig Config'))
            sync = int(self.getCmdStringFromValue('Trig Sync Mode'))
            self.dig.DAQtriggerExternalConfig(0, extSource, trigBehavior, sync)
        elif quant.name in ('Trig I/O', ):
            # get direction and sync from index of comboboxes
            direction = int(self.getCmdStringFromValue('Trig I/O'))
            self.dig.triggerIOconfig(direction)
        elif quant.name in ('Analog Trig Channel', 'Analog Trig Config', 'Trig Threshold'):
            # get trig channel
            trigCh = self.getValueIndex('Analog Trig Channel')
            mod = int(self.getCmdStringFromValue('Analog Trig Config'))
            threshold = self.getValue('Trig Threshold')
            self.dig.channelTriggerConfig(self.getHwCh(trigCh), mod, threshold)
        elif name in ('Range', 'Impedance', 'Coupling'):
            # set range, impedance, coupling at once
            rang = self.getRange(ch)
            imp = int(self.getCmdStringFromValue('Ch%d - Impedance' % (ch + 1)))
            coup = int(self.getCmdStringFromValue('Ch%d - Coupling' % (ch + 1)))
            self.dig.channelInputConfig(self.getHwCh(ch), rang, imp, coup)
        return value


    def performGetValue(self, quant, options={}):
        """Perform the Set Value instrument operation. This function should
        return the actual value set by the instrument"""
        # check if channel-specific, if so get channel + name
        if quant.name.startswith('Ch') and len(quant.name) > 6:
            ch = int(quant.name[2]) - 1
            name = quant.name[6:]
        else:
            ch, name = None, ''

        if name == 'Signal':
            if self.isHardwareLoop(options):
                return self.getSignalHardwareLoop(ch, quant, options)
            # get traces if first call
            if self.isFirstCall(options):
                # don't arm if in hardware trig mode
                self.getTraces(bArm=(not self.isHardwareTrig(options)))
            # return correct data
            value = quant.getTraceDict(self.lTrace[ch], dt=self.dt)
        else:
            # for all others, return local value
            value = quant.getValue()

        return value


    def performArm(self, quant_names, options={}):
        """Perform the instrument arm operation"""
        self._logger.debug("performArm called.")
        # make sure we are arming for reading traces, if not return
        signal_names = ['Ch%d - Signal' % (n + 1) for n in range(4)]
        signal_arm = [name in signal_names for name in quant_names]
        if not np.any(signal_arm):
            return

        ## Skip hardware looping, just arm without measuring
        self.getTraces(bArm=True, bMeasure=False)


    def getTraces(self, bArm=True, bMeasure=True, n_seq=0):
        """Get all active traces"""
        self._logger.debug("getTraces called. bArm: {}, bMeasure: {}.".format(bArm, bMeasure))

        # find out which traces to get
        lCh = []
        iChMask = 0
        for n in range(self.nCh):
            if self.getValue('Ch%d - Enabled' % (n + 1)):
                lCh.append(n)
                iChMask += 2**n
        # get current settings
        nPts = int(self.getValue('Number of samples'))
        nCyclePerCall = int(self.getValue('Records per Buffer'))
        # in hardware loop mode, ignore records and use number of sequences
        if n_seq > 0:
            nSeg = n_seq
        else:
            nSeg = int(self.getValue('Number of records'))

        nAv = int(self.getValue('Number of averages'))
        # trigger delay is in 1/sample rate
        nTrigDelay = int(round(self.getValue('Trig Delay') / self.dt))

        if bArm:
            # clear old data
            self.dig.DAQflushMultiple(iChMask)
            self.lTrace = [np.array([])] * self.nCh
            # configure trigger for all active channels
            for nCh in lCh:
                # init data
                self.lTrace[nCh] = np.zeros((nSeg * nPts))
                # channel number depens on hardware version
                ch = self.getHwCh(nCh)
                # extra config for trig mode
                if self.getValue('Trig Mode') == 'Digital trigger':
                    extSource = int(self.getCmdStringFromValue('External Trig Source'))
                    trigBehavior = int(self.getCmdStringFromValue('External Trig Config'))
                    sync = int(self.getCmdStringFromValue('Trig Sync Mode'))
                    self.dig.DAQtriggerExternalConfig(ch, extSource, trigBehavior, sync)
                    self.dig.DAQdigitalTriggerConfig(ch, extSource, trigBehavior)
                elif self.getValue('Trig Mode') == 'Analog channel':
                    digitalTriggerMode= 0
                    digitalTriggerSource = 0
                    trigCh = self.getValueIndex('Analog Trig Channel')
                    analogTriggerMask = 2**trigCh
                    self.dig.DAQtriggerConfig(ch, digitalTriggerMode, digitalTriggerSource, analogTriggerMask)
                # config daq and trig mode
                trigMode = int(self.getCmdStringFromValue('Trig Mode'))
                self.dig.DAQconfig(ch, nPts, nSeg*nAv, nTrigDelay, trigMode)
            # start acquiring data
            self.dig.DAQstartMultiple(iChMask)
        # lT.append('Start %.1f ms' % (1000*(time.perf_counter()-t0)))
        #
        # return if not measure
        if not bMeasure:
            return

        ## Measurement starts here
        
        iMeasChannel = 0 # TODO make this dynamic!
        sOutputPath = os.path.expanduser("~/Digitizer_PPG/data/data_out.npy")
        dScale = (self.getRange(iMeasChannel) / self.bitRange)
        nEvents = 1 # TODO make this dynamic!
        ## Data container
        lData = []
        ## Loop over fixed number of events
        for nn in range(nEvents):

            self._logger.debug("Watiting for event "+str(nn))

            ## TODO report progress

            ## Capture traces - only 1 channel!
            ch = self.getHwCh(iMeasChannel)
            self._logger.debug("Fetching data from DAQ...")
            data_raw  = self.DAQread(self.dig, ch, nPts, int(1000+self.timeout_ms))
            self._logger.debug("Preprocessing data")
            ## Scale raw data to convert to physical units (V)
            data = data_raw * dScale
            ## Append to list
            lData.append(data)

            ## Break if stopped from outside
            if self.isStopped():
                break

        self._logger.info("Data collection completed after "+str(nEvents)+" trigger events.")
        ## Convert to array
        aData = np.array(lData)
        ## Dump to file (with logging to check timing)
        self._logger.debug("Writing data to file...")
        np.save(sOutputPath, aData, allow_pickle=False, fix_imports=False)
        self._logger.debug("Data written to "+sOutputPath)


    def getRange(self, ch):
        """Get channel range, as voltage.  Index start at 0"""
        rang = float(self.getCmdStringFromValue('Ch%d - Range' % (ch + 1)))
        # range depends on impedance
        if self.getValue('Ch%d - Impedance' % (ch + 1)) == 'High':
            rang = rang * 2
            # special case if range is .25, 0.5, or 1, scale to 0.2, .4, .8
            if rang < 1.1:
                rang *= 0.8
        return rang


    def DAQread(self, dig, nDAQ, nPoints, timeOut):
        """Read data diretly to numpy array"""
        if dig._SD_Object__handle > 0:
            if nPoints > 0:
                data = (keysightSD1.c_short * nPoints)()
                nPointsOut = dig._SD_Object__core_dll.SD_AIN_DAQread(dig._SD_Object__handle, nDAQ, data, nPoints, timeOut)
                if nPointsOut > 0:
                    return np.frombuffer(data, dtype=np.int16, count=nPoints)
                else:
                    return np.array([], dtype=np.int16)
            else:
                return keysightSD1.SD_Error.INVALID_VALUE
        else:
            return keysightSD1.SD_Error.MODULE_NOT_OPENED


    def getSignalHardwareLoop(self, ch, quant, options):
        """Get data from round-robin type averaging"""
        (seq_no, n_seq) = self.getHardwareLoopIndex(options)
        # after getting data, pick values to return
        return quant.getTraceDict(self.reshaped_traces[ch][seq_no], dt=self.dt)


if __name__ == '__main__':
    pass
