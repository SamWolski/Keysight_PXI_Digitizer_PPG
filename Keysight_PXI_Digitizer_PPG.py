#!/usr/bin/env python
import os
import sys
import logging
from datetime import datetime
import multiprocessing

sys.path.append('C:\Program Files (x86)\Keysight\SD1\Libraries\Python')
from BaseDriver import LabberDriver, Error, IdError
import keysightSD1

import numpy as np

import queue
import threading


## multiprocessing parameters
address = ("localhost", 50020)


def generate_log_bools(n_events, total_events):
    """
    Generate a log-spaced array of `n_events` True values in a list of length 
    `total_events`
    """
    ## Generate initial log-spaced list, rounded to integers
    event_indices = np.round(
        np.logspace(0, np.log10(total_events), n_events)-1).astype(int)

    ## Remove duplicates by incrementing values
    event_indices_clean = np.zeros_like(event_indices)
    test_index = -1
    for ii, index_val in enumerate(event_indices):
        if index_val <= test_index:
            index_val = test_index + 1
        test_index = index_val
        event_indices_clean[ii] = index_val

    ## Convert to true-false array
    event_bools_list = np.array([False]*total_events)
    event_bools_list[event_indices_clean] = True

    return event_bools_list


## Microsecond formatting for logger
class MusecFormatter(logging.Formatter):
    converter=datetime.fromtimestamp
    def formatTime(self, record, datefmt=None):
        ct = self.converter(record.created)
        if datefmt:
            s = ct.strftime(datefmt)
        else:
            t = ct.strftime("%y-%m-%d %H:%M:%S")
            s = "%s.%03d" % (t, record.msecs)
        return s


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
        # logging.basicConfig(filename=log_path, filemode="a",
        #                     level=logging.DEBUG,
        #                     # format="%(asctime)s %(name)-8s: %(message)s",
        #                     # datefmt="%y-%m-%d %H:%M:%S"
        #                     )
        self._logger = logging.getLogger("Digitizer_PPG")
        self._logger.setLevel(logging.DEBUG)
        file_handler = logging.FileHandler(filename=log_path, mode="a")
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(MusecFormatter(
                                    fmt="%(asctime)s %(name)-8s: %(message)s",
                                    datefmt="%y-%m-%d %H:%M:%S.%f"))
        self._logger.addHandler(file_handler)
        self._logger.info("Logging initialized to {}".format(log_path))
        self._logger.debug("Using python version {}".format(sys.version_info))
        self._logger.debug("Python installation is at {}".format(sys.executable))
        # self._logger.debug("Imported modules (from sys.modules.keys()): {}".format(sys.modules.keys()))
        # self._logger.debug("sys.path is: {}".format(sys.path))


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
        sOutputDir = os.path.expanduser("~/Digitizer_PPG/data/")
        dScale = (self.getRange(iMeasChannel) / self.bitRange)
        bSparse = self.getValue("Log-spaced sparse event handling")
        bScaleValues = self.getValue("Scale values before saving")
        nRecords = int(self.getValue('Saved trigger events'))
        if bSparse:
            nEvents = int(self.getValue("Total trigger events"))
            ## Generate list of filtering bools to decide which datasets to save
            lSaveEventFlags = generate_log_bools(nRecords, nEvents)
        else:
            nEvents = nRecords
            lSaveEventFlags = [True]*nEvents
        self._logger.info("Number of saved/total trigger events: {}/{}".format(nRecords, nEvents))

        bUseMultithreading = self.getValue("Use multithreading")
        self._logger.info("Multithreading enabled: {}".format(bUseMultithreading))
        if bUseMultithreading:
            ## Saver thread
            self._logger.debug("Starting saver thread...")
            self.saver_queue = queue.Queue(maxsize=nRecords)
            threading.Thread(target=self.saver, args=(sOutputDir, dScale)).start()

        if bSparse:
            nRecordsPerBuffer = nRecords
            nEventsPerBuffer = nEvents
            nEventBuffers = 1
        else:
            nRecordsPerBuffer = int(self.getValue('Records per Buffer'))
            nEventsPerBuffer = nRecordsPerBuffer
            nEventBuffers = int(np.ceil(nRecords / nRecordsPerBuffer))

        file_index_counter = 0
        nRecordsPerFile = int(self.getValue("Records per File"))
        ## Loop over fixed number of events
        for bb in range(nEventBuffers):
            self._logger.debug("Event buffer "+str(bb))

            lData = []

            if bSparse:
                self._logger.debug("Expecting "+str(nRecordsPerBuffer)+" records from "+str(nEventsPerBuffer)+" events in this set.")
            else:
                ## Number of trigger events in this call; can be less for final call
                nEventsPerBuffer = min(nEventsPerBuffer, nEvents-(bb*nEventsPerBuffer))
                self._logger.debug("Expecting "+str(nEventsPerBuffer)+" events in this set.")
            for nn, bEventFlag in zip(range(nEventsPerBuffer), lSaveEventFlags):
                # ## The very first time, wait for a request from the AWG driver
                if (bb==0) and (nn==0):
                    ## If we need to wait, do it here!
                    self._logger.debug("Starting Client connection...")
                    conn = multiprocessing.connection.Client(address, authkey=b"p")
                    self._logger.debug("Sending request to AWG driver...")
                    conn.send("")
                    self._logger.debug("Listening for response from AWG driver...")
                    _ = conn.recv() # should block until the response is received
                    self._logger.debug("Received response from AWG driver; closing connection object.")
                    conn.close()
                ## The first time, we will only get here after sync
                self._logger.debug("Starting DAQ acquisition...")
                self.dig.DAQstartMultiple(iChMask)

                self._logger.debug("Waiting for event "+str(nn))

                ## TODO report progress

                ## Capture traces - only 1 channel!
                ch = self.getHwCh(iMeasChannel)
                self._logger.debug("Fetching data from DAQ...")
                data_raw  = self.DAQread(self.dig, ch, nPts, int(1000+self.timeout_ms))
                if bEventFlag:
                    self._logger.debug("Adding data record to list.")
                    ## Append to list
                    lData.append(data_raw)
                else:
                    self._logger.debug("Discarding data record.")

                ## Break if stopped from outside
                if self.isStopped():
                    break

            ## 
            if bUseMultithreading:
                self._logger.debug("Adding data to queue...")
                self.saver_queue.put(lData)
            else:

                ## Save data directly
                nNumberOfFiles = int(np.ceil(len(lData)/nRecordsPerFile))
                self._logger.debug("Saving "+str(len(lData))+" total records...")
                for nfile in range(nNumberOfFiles):
                    list_slice = slice(nfile*nRecordsPerFile, (nfile+1)*nRecordsPerFile)
                    self._logger.debug("Slicing: {}".format(list_slice))
                    self._logger.debug("Consolidating data subset into array...")
                    aData = np.array(lData[list_slice], dtype=np.int16)
                    if bScaleValues:
                        self._logger.debug("Scaling data...")
                        aData = aData * dScale
                    self._logger.debug("Writing data to file...")
                    sOutputPath = os.path.join(sOutputDir, "data_out_"+str(file_index_counter)+".npy")
                    np.save(sOutputPath, aData, allow_pickle=False, fix_imports=False)
                    self._logger.info("Data written to "+sOutputPath)
                    file_index_counter += 1
            ## 

            ## Break if stopped from outside
            if self.isStopped():
                break

        self._logger.info("Data collection completed after "+str(nEvents)+" trigger events.")
        if bUseMultithreading:
            self.saver_queue.put("exit")
            self.saver_queue.join()



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
                self._logger.debug("Points received: "+str(nPointsOut))
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


    ## Multithreading

    def saver(self, sOutputDir, dScale):
        file_index = 0
        for lData in iter(self.saver_queue.get, 'exit'):
            self._logger.debug("Received buffer "+str(file_index))
            self._logger.debug("Consolidating data list into array...")
            aDataRaw = np.array(lData, dtype=np.int16)
            self._logger.debug("Scaling data...")
            aData = aDataRaw * dScale
            self._logger.debug("Writing data to file...")
            sOutputPath = os.path.join(sOutputDir, "data_out_"+str(file_index)+".npy")
            np.save(sOutputPath, aData, allow_pickle=False, fix_imports=False)
            self._logger.info("Data written to "+sOutputPath)
            self.saver_queue.task_done()
            file_index += 1
        ## After receiving "exit" call
        self.saver_queue.task_done()



if __name__ == '__main__':
    pass
