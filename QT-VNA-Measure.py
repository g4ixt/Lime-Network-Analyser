import sys
import numpy
from PyQt5 import QtWidgets
import pyqtgraph
from pyLMS7002Soapy import pyLMS7002Soapy as pyLMSS
import Qt_designer_VNA_Gui

calThreshold = 500  # RSSI threshold to trigger RX DC cal. 250 is ~50% slower than 500

#  LMS7002M acronyms and features #

#  TSP Transceiver Signal Processsor
#  LNA receiver Low Noise Amplifier - before first mixer
#  TIA receiver Trans Impedance Amplifier - after first mixer, before low pass filter
#  PGA receiver Programmable Gain Amplifier - after low pass filter, before ADC
#  RSSI Received Signal Strength Indicator
#  PAD transmitter Power Amplifier Driver - final stage
#  TBB transmitter filter stage controls
#  DLB Digital Loopback.  Tx signal IQ fed back from Tx port directly to Rx port
#  BBLB Baseband Loopback.  Tx signal IQ after LPF fed back to input of Rx LPF
#  RFLB RF Loopback. RF Tx signal from PAD output fed to input of Rx LNA
#  SXR Receive Synthesiser
#  SXT Transmit Synthesiser
#  TDD Mode = 1. The receive IQ mixer uses the signal from the transmit synthesiser


class Measurement():
    '''Create measurements of Amplitude and Phase for lists of n frequencies (MHz)'''

    def Analyse(self, Calibration, Rx, measType):
        limeSDR = pyLMSS.pyLMS7002Soapy(0)
        lms7002 = limeSDR.LMS7002
        # Measure received signal power and phase. Phase is not plotted (yet) but can be saved to file.
        self.measType = measType
        self.res = []
        self.resPhase = []
        self.pgaGains = []  # stores pga gains set per Freq during calibration
        self.lnaGains = []  # stores lna gains set per Freq during calibration
        self.refPhase = 0
        power = []
        freq = []

        startFreq, endFreq, nFreq, centreFreq, spanFreq = getFreq()
        nFreq += 1  # to give equal frequency spacing around centre freq
        self.freqs = numpy.linspace(startFreq, endFreq, nFreq)

        hardware.freqDepVar(startFreq)
        LNA = hardware.lna
        TxTSP = lms7002.TxTSP[hardware.txChan]
        TRF = lms7002.TRF[hardware.txChan]
        setTransceiver(lms7002, Rx, startFreq)

        #  valid MAC values are [1,2,'A','B','R','RX','T','TX']. Tells MCU which channel to use for trx, tx, rx
        #  synthesisers SXT and SXR share register addresses so channel is identified by MAC setting
        lms7002.MAC = Rx

        for i in range(0, len(self.freqs)):
            lms7002.verbose = 0  # controls logging level.  Set to 1000 for more detail
            f = self.freqs[i] * 1e6

            # set freq and cal rx DC offset. Both Tx and Rx since rx is using tx PLL
            lms7002.SX['T'].setFREQ(f)
            lms7002.SX['T'].PD_LOCH_T2RBUF = 0  # set to use tx PLL (TDD Mode)

            if Calibration != '':  # Set gains for DUT measurement to the calibrated values
                pgaGain = Calibration.pgaGains[i]
                lnaGain = Calibration.lnaGains[i]
                lms7002.RBB[Rx].G_PGA_RBB = pgaGain
                lms7002.RFE[Rx].G_LNA_RFE = lnaGain
            else:
                # optimise Rx gain for best dynamic range, return the values and append them to cal lists
                pgaGain, lnaGain = adjustRxGain(lms7002, Rx, i)
                self.pgaGains.append(pgaGain)
                self.lnaGains.append(lnaGain)

            # set transmit and receive for testing? loopback?
            syncPhase(lms7002, Rx)

            # Check residual RSSI (DC offset?) at the set rx gain
            TRF.PD_TXPAD_TRF = 'OFF'  # Transmit power amplifier
            calRSSI = lms7002.RxTSP[Rx].RSSI
            if calRSSI > calThreshold:
                lms7002.calibration.rxDCLO(Rx, LNA, lnaGain=lnaGain, pgaGain=pgaGain)  # takes about 0.9s
                calRSSI = lms7002.RxTSP[Rx].RSSI
            TRF.PD_TXPAD_TRF = 'ON'

            # get the (averaged) RSSI value from the LimeSDR
            TxTSP.CMIX_BYP = 'USE'
            lms7002.RxTSP[Rx].GC_BYP = 'USE'  # turn on gain cor
            rssi = 1.0 * mcuRSSI(lms7002)
            TxTSP.CMIX_BYP = 'BYP'
            lms7002.RxTSP[Rx].GC_BYP = 'BYP'
            lms7002.RxTSP[Rx].GCORRQ = 2047

            # add the RSSI value to the magnitude list 'res', in position i
            self.res.append(rssi)
            DutPower = 20 * numpy.log10(rssi)

            # measure phase for freq i and append to list.
            if ui.Phase.isChecked():
                if Calibration == "":
                    if i == 0:
                        self.refPhase = mcuPhase(lms7002, Rx)
                    phase = mcuPhase(lms7002, Rx) - self.refPhase  # set cal phase ref=zero at start freq
                else:
                    phase = mcuPhase(lms7002, Rx) - Calibration.refPhase
                self.resPhase.append(phase)
            else:
                self.resPhase.append(0)

            # plot the results on the GUI graphwidget and update the progress indicators.
            freq.append(self.freqs[i])
            progress = int((i+1)*100/len(self.freqs))
            if Calibration == '':
                CalPower = 0
            else:
                CalPower = 20 * numpy.log10(Calibration.res[i])
            power.append(DutPower-CalPower)
            if measType == 'ReturnLoss':
                ui.calShortProgress.setValue(progress)
                rlCurve.setData(freq, power)
            else:
                ui.calThroughProgress.setValue(progress)
                throCurve.setData(freq, power)
            pyqtgraph.QtGui.QApplication.processEvents()  # force GUI update

        if ui.SaveBox.isChecked():
            reference = str(self)  # a unique filename reference for the measurement instance
            writeDataFile(reference[32:-1], self.measType, self.freqs, self.res, self.resPhase)


class Marker():
    '''Create amplitude and frequency markers using infinite lines'''

    def __init__(self, markerType, markerValue, labelPosition):
        self.Type = markerType
        self.Value = markerValue
        self.lPos = labelPosition
        self.line = ui.graphWidget.addLine()

    def createLine(self):
        if Marker1.Value == 0:  # first run, so set freq markers to GUI values
            Marker1.Value = ui.Marker1.value()
            Marker2.Value = ui.Marker2.value()
            Marker3.Value = ui.Marker3.value()
        markerPen = pyqtgraph.mkPen(color='g', width=0.5)
        if self.Type == 'freq':
            self.line = ui.graphWidget.addLine(x=self.Value, movable=True, pen=markerPen, label="{value:.2f}")
        else:
            self.line = ui.graphWidget.addLine(y=self.Value, movable=True, pen=markerPen, label="{value:.2f}")
        self.line.label.setPosition(self.lPos)
        triggerMarkers()

    def updateLine(self):
        # update line value and position to SpinBox settings
        Marker1.Value = ui.Marker1.value()
        Marker2.Value = ui.Marker2.value()
        Marker3.Value = ui.Marker3.value()
        Marker4.Value = ui.Marker4.value()
        Marker5.Value = ui.Marker5.value()
        self.line.setValue(self.Value)

    def updateSpinBox(self):
        # update spinboxes to values markers were dragged to
        ui.Marker1.setValue(Marker1.line.value())
        ui.Marker2.setValue(Marker2.line.value())
        ui.Marker3.setValue(Marker3.line.value())
        ui.Marker4.setValue(Marker4.line.value())
        ui.Marker5.setValue(Marker5.line.value())
        self.Value = self.line.value()


class setSDR():
    '''Set values for Lime-Mini or Lime-USB and its frequency-dependent settings'''

    def setVariables(self):
        #  set variables to the correct values for Lime-Mini or Lime-USB
        limeSDR = pyLMSS.pyLMS7002Soapy(0)
        if limeSDR.boardName == 'LimeSDRMini':
            self.sdrName = 'LimeSDR-Mini'
            self.fRef = 40e6
            self.iAmp = 5
            self.band1 = 1
            self.band2 = 0  # not relevant for Mini but presume still need to set it
            self.lnaGain = 6
            self.txChan = 'A'
        else:
            self.sdrName = 'LimeSDR-USB'
            self.fRef = 30.72e6
            self.iAmp = 20  # for TBB.CG_IAMP_TBB - controls front-end gain of TBB.  Seems to have little effect.
            self.band1 = 1  # for TRF.SEL_BAND1_TRF
            self.band2 = 0  # for TRF.SEL_BAND2_TRF
            self.lnaGain = 6  # was 8.  1 to 15 allowed
            self.pgaGain = 1
            self.lna = 'LNAL'  # not reqd?

    def freqDepVar(self, startFreq):
        limeSDR = pyLMSS.pyLMS7002Soapy(0)
        #  <= 800MHz, tx pwr is +10dB for Band1 vs Band2.  It then falls ~'linearly' to ~equal at ~2100MHz.
        #  >= 2100MHz tx pwr is between 0dB and +15dB for Band 2.  At 2400MHz about +4dB and very peaky at 2900MHz
        if self.sdrName == 'LimeSDR-USB':
            if ui.TxAorB.value() == 0:
                self.txChan = 'A'
            else:
                self.txChan = 'B'
            if startFreq < 1500:
                self.lna = 'LNAL'
                self.band1 = 1
                self.band2 = 0
            elif startFreq >= 1500 and startFreq < 2100:
                self.lna = 'LNAH'
                self.band1 = 1
                self.band2 = 0
            else:
                self.lna = 'LNAH'
                self.band1 = 0
                self.band2 = 1
        else:
            limeSDR.configureAntenna(startFreq)  # doesn't this do all that above for the usb as well - see pySoapy?

################################################################
# MCU related


def mcuProgram(lms7002):
    # Load the MCU firmware for measuring RSSI
    logTxt("Loading MCU program...\t", end="")
    mcu = lms7002.mSPI
    mcu.loadHex("vna.hex")
    # Check MCU firmware
    mcu.P0 = 0
    mcu.P0 = 0xFF
    firmwareID = mcu.P1 - 0x80
    if firmwareID != 0x31:
        logTxt("Wrong firmware ID : " + str(firmwareID) + ", expected 49")
        sys.exit(1)
    else:
        logTxt("OK (Firmware ID = " + str(firmwareID) + ")")
    mcu.P0 = 0


def mcuRSSI(lms7002):
    # Read averaged RSSI from MCU
    mcu = lms7002.mSPI
    mcu.SPISW_CTRL = 'MCU'
    mcu.P0 = 1
    while mcu.P1 == 0xFF:
        pass
    mcu.P0 = 0x12
    RSSI = mcu.P1 * 1.0
    mcu.P0 = 0x11
    RSSI = mcu.P1 * 1.0 + RSSI * 256.0
    mcu.P0 = 0x10
    RSSI = mcu.P1 * 1.0 + RSSI * 256.0
    mcu.SPISW_CTRL = 'BB'
    return RSSI


def mcuPhase(lms7002, Rx):
    # Phase is measured by setting I channel gain to zero using Rx TSP gain corrector
    # The Tx phase is then adjusted to minimise RSSI which occurs when Tx phase = Rx phase
    mcu = lms7002.mSPI
    RxTSP = lms7002.RxTSP[Rx]
    RxTSP.GC_BYP = 'USE'
    RxTSP.GCORRI = 0

    mcu.SPISW_CTRL = 'MCU'
    mcu.P0 = 2
    while mcu.P1 == 0xFF:
        pass
    mcu.P0 = 0x11
    phase = 1.0 * mcu.P1
    mcu.P0 = 0x10
    phase = mcu.P1 + phase * 256.0
    mcu.SPISW_CTRL = 'BB'
    if phase > 2 ** 15:
        phase = -(2 ** 16) + phase
    phase = 180.0 * phase / (1.0 * 0x6487)
    RxTSP.GC_BYP = 'BYP'
    RxTSP.GCORRI = 2047
    return phase

#################################################
# Auxiliary functions


def logTxt(text, end="\n"):
    print(text, end=end)
    sys.stdout.flush()


def syncPhase(lms7002, Rx):
    TRF = lms7002.TRF[hardware.txChan]
    RFE = lms7002.RFE[Rx]
    SXT = lms7002.SX['T']
    SXT.PD_FDIV = 1  # power down forward freq divider of the LO chain
    TRF.PD_TLOBUF_TRF = 1  # power down tx LO buffer
    RFE.PD_QGEN_RFE = 1  # power down RXFE quadrature LO generator
    TRF.PD_TLOBUF_TRF = 0  # power up tx LO buffer
    RFE.PD_QGEN_RFE = 0  # power up RXFE quadrature LO generator
    SXT.PD_FDIV = 0  # power up forward freq divider of the LO chain


def adjustRxGain(lms7002, Rx, i):  # returns pga and lna gains for optimum dynamic range.
    RBB = lms7002.RBB[Rx]
    RFE = lms7002.RFE[Rx]
    RxTSP = lms7002.RxTSP[Rx]
    TxTSP = lms7002.TxTSP[hardware.txChan]

    TxTSP.CMIX_BYP = 'USE'
    RxTSP.GC_BYP = 'USE'
    RxTSP.GCORRQ = 0

    if ui.setGain.isChecked():
        #if i == 0:  # set LNA gain at the start frequency only
        lnaGain = 1
        RFE.G_LNA_RFE = 1
        RBB.G_PGA_RBB = 24  # set PGA gain with a little headroom
        rssi = mcuRSSI(lms7002)
        while rssi < 50e3 and lnaGain < 15:
            rssi = mcuRSSI(lms7002)
            lnaGain += 2
            RFE.G_LNA_RFE = lnaGain
            ui.RSSI.setValue(int(rssi))
            ui.lnaGain.setValue(lnaGain)
            app.processEvents()
            hardware.lnaGain = lnaGain  # keep value for next call of adjustRxGain in Analyse()
            if mcuRSSI(lms7002) >= 50e3:
                break
    else:
        lnaGain = ui.lnaGain.value()

    pgaGain = 0
    pgaStep = 16
    lnaGain = hardware.lnaGain

    while pgaStep > 0:
        RBB.G_PGA_RBB = pgaGain + pgaStep
        RFE.G_LNA_RFE = lnaGain
        if mcuRSSI(lms7002) < 50e3:
            pgaGain += pgaStep
            #  logTxt(str(pgaGain) + '\t' + str(pgaStep) + '\t' + str(mcuRSSI(lms7002)) + '\n')
            ui.RSSI.setValue(int(mcuRSSI(lms7002)))
            ui.pgaGain.setValue(pgaGain)
            app.processEvents()
        pgaStep = int(pgaStep / 2)

    TxTSP.CMIX_BYP = 'BYP'
    RxTSP.GC_BYP = 'BYP'
    RxTSP.GCORRQ = 2047
    I = 0x7FFF
    Q = 0x8000
    TxTSP.loadDCIQ(I, Q)
    return pgaGain, lnaGain


def writeDataFile(measName, measType, freqs, res, resPhase):
    # For compatibility with original 'calculateVNA'. (Filename needs to be amended after.)
    outFileName = 'vna_' + measName + '_DUT_' + measType + '.txt'
    outFile = open(outFileName, 'w')
    txtRes = "# f, coupled power\n"
    for i in range(0, len(res)):
        f = freqs[i]
        y = res[i]
        phase = resPhase[i]
        txtRes += str(f) + '\t' + str(y) + '\t' + str(phase) + '\t' + '\n'
    outFile.write(txtRes)
    outFile.close()


def setTransceiver(lms7002, Rx, startFreq):
    #  set rx and tx for the start frequency
    TBB = lms7002.TBB[hardware.txChan]  # does this also depend on MAC setting?
    TRF = lms7002.TRF[hardware.txChan]
    RxTSP = lms7002.RxTSP[Rx]
    TxTSP = lms7002.TxTSP[hardware.txChan]
    if hardware.txChan == 'A':
        TxNCO = lms7002.NCO["TXA"]
    else:
        TxNCO = lms7002.NCO["TXB"]

    lms7002.fRef = hardware.fRef  # set correct clock frequency for -USB or -Mini
    lms7002.MIMO = 'MIMO'

    # Initial configuration
    ui.InitialisedMessage.setText("Tuning Clock")
    app.processEvents()
    lms7002.CGEN.setCLK(300e6)  # set clock to 300MHz
    hardware.freqDepVar(startFreq)
    ui.InitialisedMessage.setText("Tuning SXT")
    app.processEvents()

    startFreq = float(startFreq * 1e6)
    lms7002.SX['T'].setFREQ(startFreq)

    # Make ADC and DAC clocks equal
    ui.InitialisedMessage.setText("Setting up RSSI")
    app.processEvents()
    lms7002.CGEN.EN_ADCCLKH_CLKGN = 0  # set ADC clock to F_CLKH and DAC clock to F_CLKL
    lms7002.CGEN.CLKH_OV_CLKL_CGEN = 2

    # configure Rx
    RxTSP.GCORRQ = 2047  # set gain corrector to maximum
    RxTSP.GCORRI = 2047
    RxTSP.AGC_MODE = 'RSSI'
    RxTSP.AGC_BYP = 'USE'
    RxTSP.RSSI_MODE = 'RSSI'

    # Tx calibration
    TBB.CG_IAMP_TBB = hardware.iAmp
    TxTSP.TSGMODE = 'DC'
    TxTSP.INSEL = 'TEST'
    TxTSP.CMIX_BYP = 'BYP'
    TxTSP.GFIR1_BYP = 'BYP'
    TxTSP.GFIR2_BYP = 'BYP'
    TxTSP.GFIR3_BYP = 'BYP'
    TxTSP.GC_BYP = 'BYP'
    TxTSP.DC_BYP = 'BYP'
    TxTSP.PH_BYP = 'BYP'
    I = 0x7FFF
    Q = 0x8000
    TxTSP.loadDCIQ(I, Q)

    NCOfreq = 50e3  # set Numerically Controlled Oscillator to 50kHz.
    TxNCO.MODE = 0
    TxNCO.setNCOFrequency(0, NCOfreq)
    TxNCO.SEL = 0

    TRF.EN_LOOPB_TXPAD_TRF = 'OFF'  # TxPAD loopback disabled
    TRF.L_LOOPB_TXPAD_TRF = 0  # set loopback loss to zero
    TRF.PD_TLOBUF_TRF = 0  # enable TX LO Buffer
    TRF.LOSS_MAIN_TXPAD_TRF = 0  # Sets TxPAD o/p power by adjusting loss in range 0 to 31.  Max power = 0
    TRF.SEL_BAND1_TRF = hardware.band1
    TRF.SEL_BAND2_TRF = hardware.band2

    lms7002.SX['R'].EN_G = 0
    lms7002.SX['R'].EN_DIR = 0
    lms7002.SX['T'].PD_LOCH_T2RBUF = 0  # Both RX and TX use the TX PLL

    #  initial calibration of Rx DC with TxPAD off
    ui.InitialisedMessage.setText("Rx DC calibration")
    app.processEvents()
    TRF.PD_TXPAD_TRF = 'OFF'
    lms7002.calibration.rxDCLO(Rx, hardware.lna, lnaGain=15, pgaGain=31)
    TRF.PD_TXPAD_TRF = 'ON'

    #  Tells MCU which channel to use for trx, tx, rx since synths SXT and SXR share register addresses
    lms7002.MAC = Rx
    ui.InitialisedMessage.setText("Ready")


def ConnectSDR():
    # Connect to SDR and initialise
    limeSDR = pyLMSS.pyLMS7002Soapy(0)
    lms7002 = limeSDR.LMS7002
    # lms7002.verbose = 1000
    hardware.setVariables()
    ui.ConnectButton.setText(hardware.sdrName)
    ui.InitialisedMessage.setText("Loading VNA.hex to MCU")
    app.processEvents()
    mcuProgram(lms7002)  # Load vna.hex to MCU SRAM for measuring RSSI
    connectedButtons(True)
    ui.InitialisedMessage.setText("Ready")

##############################################################################

# interfaces to the GUI


def connectedButtons(enable):
    ui.CalShortButton.setEnabled(enable)
    ui.CalThroughButton.setEnabled(enable)
    ui.centreFreq.setEnabled(enable)
    ui.spanFreq.setEnabled(enable)
    ui.numPoints.setEnabled(enable)
    ui.Marker1.setEnabled(enable)
    ui.Marker2.setEnabled(enable)
    ui.Marker3.setEnabled(enable)
    ui.Marker4.setEnabled(enable)
    ui.Marker5.setEnabled(enable)


def measureButtons(enable):
    ui.MeasureRLButton.setEnabled(enable)
    ui.MeasureThroughButton.setEnabled(enable)
    if not enable:
        ui.calShortProgress.setValue(0)
        ui.calThroughProgress.setValue(0)


def calReturnLoss():
    ui.graphWidget.setYRange(-100, 100)
    ui.calShortProgress.setValue(0)
    startFreq, endFreq, nFreq, centreFreq, spanFreq = getFreq()
    short.Analyse('', 'A', 'ReturnLoss')
    ui.MeasureRLButton.setEnabled(True)


def calThroughLoss():
    ui.graphWidget.setYRange(-100, 100)
    ui.calThroughProgress.setValue(0)
    startFreq, endFreq, nFreq, centreFreq, spanFreq = getFreq()
    through.Analyse('', 'B', 'ThroughLoss')
    ui.MeasureThroughButton.setEnabled(True)


def measReturnLoss():
    ui.graphWidget.setYRange(-35, 5)
    while ui.Repeat.isChecked():
        DutRefl.Analyse(short, 'A', 'ReturnLoss')
        pyqtgraph.QtGui.QApplication.processEvents()
        if not ui.Repeat.isChecked(): break
    else:
        DutRefl.Analyse(short, 'A', 'ReturnLoss')


def measThroughLoss():
    ui.graphWidget.setYRange(-35, 5)
    while ui.Repeat.isChecked():
        DutThro.Analyse(through, 'B', 'ThroughLoss')
        pyqtgraph.QtGui.QApplication.processEvents()
        if not ui.Repeat.isChecked(): break
    else:
        DutThro.Analyse(through, 'B', 'ThroughLoss')


def getFreq():
    centreFreq = ui.centreFreq.value()
    spanFreq = ui.spanFreq.value()
    startFreq = int(centreFreq-(spanFreq/2))
    endFreq = int(centreFreq+(spanFreq/2))
    nFreq = ui.numPoints.value()
    return startFreq, endFreq, nFreq, centreFreq, spanFreq


def triggerMarkers():
    Marker1.line.sigPositionChanged.connect(Marker1.updateSpinBox)
    Marker2.line.sigPositionChanged.connect(Marker2.updateSpinBox)
    Marker3.line.sigPositionChanged.connect(Marker3.updateSpinBox)
    Marker4.line.sigPositionChanged.connect(Marker4.updateSpinBox)
    Marker5.line.sigPositionChanged.connect(Marker5.updateSpinBox)


def freqChanged():
    startFreq, endFreq, nFreq, centreFreq, spanFreq = getFreq()
    ui.graphWidget.setXRange(startFreq, endFreq)
    markerOffset = int(spanFreq/10)
    ui.Marker1.setValue(startFreq+markerOffset)
    ui.Marker2.setValue(centreFreq)
    ui.Marker3.setValue(endFreq-markerOffset)
    Marker1.updateLine()
    Marker2.updateLine()
    Marker3.updateLine()
    measureButtons(False)


def initialiseButtons():
    ui.CalShortButton.setEnabled(False)


# Create measurements and markers. Launch and respond to the GUI
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Qt_designer_VNA_Gui.Ui_MainWindow()
    ui.setupUi(MainWindow)
    ui.graphWidget.showGrid(x=True, y=True)
    ui.graphWidget.setLabel('bottom', 'Frequency MHz')
    ui.graphWidget.setLabel('left', 'Relative Signal Power dB')
    ui.graphWidget.setXRange(1250, 1350)
    ui.graphWidget.setYRange(-35, 5)
    ui.graphWidget.addLegend(offset=2)
    rlCurve = ui.graphWidget.plot([], [], name='Return Loss', pen='y', width=5)
    throCurve = ui.graphWidget.plot([], [], name='Through Loss', pen='c', width=3)

    # instantiate measurements, markers, and hardware-dependent variables
    hardware = setSDR()
    short = Measurement()
    through = Measurement()
    DutRefl = Measurement()
    DutThro = Measurement()

    Marker1 = Marker('freq', 0, 0.99)
    Marker2 = Marker('freq', 0, 0.97)
    Marker3 = Marker('freq', 0, 0.95)
    Marker4 = Marker('ampl', -5.0, 0.03)
    Marker5 = Marker('ampl', -15.0, 0.08)

    Marker1.createLine()
    Marker2.createLine()
    Marker3.createLine()
    Marker4.createLine()
    Marker5.createLine()

    # call functions when the GUI buttons etc are clicked
    ui.ConnectButton.clicked.connect(getFreq)
    ui.ConnectButton.clicked.connect(ConnectSDR)
    ui.CalShortButton.clicked.connect(calReturnLoss)
    ui.CalThroughButton.clicked.connect(calThroughLoss)
    ui.MeasureRLButton.clicked.connect(measReturnLoss)
    ui.MeasureThroughButton.clicked.connect(measThroughLoss)
    ui.centreFreq.valueChanged.connect(freqChanged)
    ui.spanFreq.valueChanged.connect(freqChanged)
    ui.numPoints.valueChanged.connect(freqChanged)
    ui.Marker1.valueChanged.connect(Marker1.updateLine)
    ui.Marker2.valueChanged.connect(Marker2.updateLine)
    ui.Marker3.valueChanged.connect(Marker3.updateLine)
    ui.Marker4.valueChanged.connect(Marker4.updateLine)
    ui.Marker5.valueChanged.connect(Marker5.updateLine)

    MainWindow.show()
    sys.exit(app.exec_())
