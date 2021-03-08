import sys
import numpy
from PyQt5 import QtWidgets
import pyqtgraph
from pyLMS7002Soapy import pyLMS7002Soapy as pyLMSS
import Qt_designer_VNA_Gui

limeSDR = pyLMSS.pyLMS7002Soapy(0)
lms7002 = limeSDR.LMS7002

calThreshold = 500  # RSSI threshold to trigger RX DC cal. 250 is ~50% slower than 500
# TxChan = 'A'

#  LMS7002M acronyms and features #

#  TSP Transceiver Signal Processsor
#  LNA receiver Low Noise Amplifier - before first mixer
#  TIA receiver Trans Impedance Amplifier - after first mixer, before low pass filter
#  PGA receiver Programmable Gain Amplifier - after low pass filter, before ADC
#  RSSI Received Signal Strength Indicator
#  PAD transmitter Power Amplifier Driver - final stage
#  DLB Digital Loopback.  Tx signal IQ fed back from Tx port directly to Rx port
#  BBLB Baseband Loopback.  Tx signal IQ after LPF fed back to input of Rx LPF
#  RFLB RF Loopback. RF Tx signal from PAD output fed to input of Rx LNA
#  SXR Receive Synthesiser
#  SXT Transmit Synthesiser
#  TDD Mode = 1. The receive IQ mixer uses the signal from the transmit synthesizer

# CG_IAMP_TBB_(1, 2)[5:0]: This controls the front-end gain of the TBB. For a given
# gain value, this control value varies with the set TX mode. After resistance
# calibration, the following table gives the nominal values for each frequency setting.
# However, this table is to be updated and corrected after calibration. Default: 37
# Low Band:
# 5 – when 2.4MHz
# 7 – when 2.74MHz
# 12 – when 5.5MHz
# 18 – when 8.2MHz
# 24 – when 11MHz
# High Band:
# 18 – when 18.5MHz
# 37 – when 38MHz
# 54 – when 54MHz



class Measurement():
    '''Create measurements of Amplitude and Phase for lists of n frequencies (MHz)'''

    def Analyse(self, Calibration, Rx, measType):
        # Measure received signal power and phase. Phase is not plotted (yet) but can be saved to file.
        TxTSP = lms7002.TxTSP[hardware.txChan]
        TRF = lms7002.TRF[hardware.txChan]
        cal = lms7002.calibration
        self.measType = measType
        self.res = []
        self.resPhase = []
        self.pgaGains = []  # stores pga gains set per Freq during calibration
        self.lnaGains = []  # stores lna gains set per Freq during calibration
        self.refPhase = 0
        calRSSIbefore = []
        calRSSIafter = []
        power = []
        freq = []

        startFreq, endFreq, nFreq, centreFreq, spanFreq = getFreq()
        nFreq += 1  # to give equal frequency spacing around centre freq
        self.freqs = numpy.linspace(startFreq, endFreq, nFreq)
        hardware.freqDepVar(startFreq)
        LNA = hardware.lna

        #  initial calibration of Rx DC
        TRF.PD_TXPAD_TRF = 1  # Turn off tx power amplifier while calibrating RX DC
        cal.rxDCLO(Rx, hardware.lna, lnaGain=hardware.lnaGain, pgaGain=16)  # Calibrate RX DC
        TRF.PD_TXPAD_TRF = 0  # Turn on tx power amplifier

        #  valid MAC values are [1,2,'A','B','R','RX','T','TX']. Tells MCU which channel to use for trx, tx, rx
        #  synthesisers SXT and SXR share register addresses so channel is identified by MAC setting
        lms7002.MAC = Rx

        for i in range(0, len(self.freqs)):
            f = self.freqs[i] * 1e6

            if Calibration != '':  # Set gains for DUT measurement to the calibrated values
                pgaGain = Calibration.pgaGains[i]
                lnaGain = Calibration.lnaGains[i]
                lms7002.RBB[Rx].G_PGA_RBB = pgaGain
                lms7002.RFE[Rx].G_LNA_RFE = lnaGain

            # set freq and cal rx DC offset. Both Tx and Rx since rx is using tx PLL
            lms7002.verbose = 1000
            lms7002.SX['T'].setFREQ(f)
            lms7002.SX['T'].PD_LOCH_T2RBUF = 0  # set to use tx PLL (TDD Mode)

            # set transmit and receive for testing? loopback?
            syncPhase(lms7002, Rx)

            if Calibration == "":
                # optimise Rx gain for best dynamic range, return the values and append them to lists
                pgaGain, lnaGain = adjustRxGain(lms7002, Rx)  # fn doesn't change lnagain at all
                self.pgaGains.append(pgaGain)
                self.lnaGains.append(lnaGain)

            # Check residual RSSI (DC offset?) at the set rx gain
            TRF.PD_TXPAD_TRF = 1  # Turn off transmit power amplifier
            calRSSI = lms7002.RxTSP[Rx].RSSI  # tests
            calRSSIbefore.append(calRSSI)
            if calRSSI > calThreshold:
                cal.rxDCLO(Rx, LNA, lnaGain=lnaGain, pgaGain=pgaGain)  # takes about 0.9s
                calRSSI = lms7002.RxTSP[Rx].RSSI
            calRSSIafter.append(calRSSI)  # tests
            TRF.PD_TXPAD_TRF = 0  # Turn on transmit power amplifier

            # get the (average of 3) RSSI value from the LimeSDR
            TxTSP.CMIX_BYP = 'USE'
            lms7002.RxTSP[Rx].GC_BYP = 'USE'  # turn on gain corrector block
            lms7002.RxTSP[Rx].GCORRQ = 0  # set Q channel gain to zero.
            rssi = 1.0 * mcuRSSI()
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
                        self.refPhase = mcuPhase(Rx)
                    phase = mcuPhase(Rx) - self.refPhase  # set cal phase ref=zero at start freq
                else:
                    phase = mcuPhase(Rx) - Calibration.refPhase
                self.resPhase.append(phase)
            else:
                self.resPhase.append(0)

            # plot the results on the GUI graphwidget and update the progress indicators.
            freq.append(self.freqs[i])
            progress = int((i+1)*100/len(self.freqs))
            if Calibration == '':
                CalPower = 20 * numpy.log10(calRSSI)  # set to residual value so that log10 is finite
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
            writeDataFile(reference[32:-1], self.measType, self.freqs, self.res, self.resPhase, calRSSIbefore, calRSSIafter)


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


class sdrType():
    '''Set values for Lime-Mini or Lime-USB and its frequency-dependent settings'''

    def __init__(self):
        self.fRef = 30.72e6
        self.iAmp = 8  # for TBB.CG_IAMP_TBB - used to set the Tx gain? was 15
        self.band1 = 1  # for TRF.SEL_BAND1_TRF
        self.band2 = 0  # for TRF.SEL_BAND2_TRF
        self.lnaGain = 4  # was 8.  1 to 15 allowed
        self.lna = 'LNAL'
        self.sdrName = 'LimeSDR-USB'
        self.txChan = 'A'

    def setVariables(self):
        #  set variables to the correct values for Lime-Mini or Lime-USB
        limeSDR = pyLMSS.pyLMS7002Soapy(0)
        if limeSDR.boardName == 'LimeSDRMini':
            self.sdrName = 'LimeSDR-Mini'
            self.fRef = 40e6
            self.iAmp = 15  # was 15
            self.band1 = 1
            self.band2 = 0  # not relevant for Mini but presume still need to set it
            self.lnaGain = 6
        else:
            if ui.TxAorB.value() == 0:
                self.txChan = 'A'
            else:
                self.txChan = 'B'

    def freqDepVar(self, startFreq):
        #  <= 800MHz, tx pwr is +10dB for Band1 vs Band2.  It then falls ~'linearly' to ~equal at ~2100MHz.
        #  >= 2100MHz tx pwr is between 0dB and +15dB for Band 2.  At 2400MHz about +4dB and very peaky at 2900MHz
        if self.sdrName == 'LimeSDR-USB':
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
            limeSDR.configureAntenna(startFreq)

################################################################
# MCU related


def mcuProgram():
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


def mcuRSSI():
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


def mcuPhase(Rx):
    # Use MCU to determine the phase
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


def adjustRxGain(lms7002, Rx):
    # returns pga and lna gains for optimum dynamic range. No, only the pgaGain? 900MHz vers lnagain is different?
    RBB = lms7002.RBB[Rx]
    RFE = lms7002.RFE[Rx]
    RxTSP = lms7002.RxTSP[Rx]
    TxTSP = lms7002.TxTSP[hardware.txChan]

    TxTSP.CMIX_BYP = 'USE'
    RxTSP.GC_BYP = 'USE'
    RxTSP.GCORRQ = 0
    pgaGain = 0
    pgaStep = 16
    lnaGain = hardware.lnaGain

    while pgaStep > 0:
        RBB.G_PGA_RBB = pgaGain + pgaStep
        RFE.G_LNA_RFE = lnaGain
        if mcuRSSI() < 50e3:
            pgaGain += pgaStep
            logTxt(str(pgaGain) + '\t' + str(pgaStep) + '\t' + str(mcuRSSI()) + '\n')
        pgaStep = int(pgaStep / 2)

    RBB.G_PGA_RBB = pgaGain
    RFE.G_LNA_RFE = lnaGain

    TxTSP.CMIX_BYP = 'BYP'
    RxTSP.GC_BYP = 'BYP'
    RxTSP.GCORRQ = 2047

    I = 0x7FFF
    Q = 0x8000
    TxTSP.loadDCIQ(I, Q)
    return pgaGain, lnaGain


def writeDataFile(measName, measType, freqs, res, resPhase, before, after):
    # For compatibility with original 'calculateVNA'. (Filename needs to be amended after.)
    outFileName = 'vna_' + measName + '_DUT_' + measType + '.txt'
    outFile = open(outFileName, 'w')
    txtRes = "# f, coupled power\n"
    for i in range(0, len(res)):
        f = freqs[i]
        y = res[i]
        phase = resPhase[i]
        rssib = before[i]
        rssia = after[i]
        txtRes += str(f) + '\t' + str(y) + '\t' + str(phase) + '\t' + str(rssib) + '\t' + str(rssia) + '\n'
    outFile.write(txtRes)
    outFile.close()


def ConnectSDR():
    # Connect to SDR and initialise
    # Set up the shortcuts to pySoapy modules
    lms7002.verbose = 1000
    hardware.setVariables()
    TBB = lms7002.TBB[hardware.txChan]  # does this also depend on MAC setting?
    TRF = lms7002.TRF[hardware.txChan]
    RxTSPA = lms7002.RxTSP['A']
    RxTSPB = lms7002.RxTSP['B']
    TxTSP = lms7002.TxTSP[hardware.txChan]
    if hardware.txChan == 'A':
        TxNCO = lms7002.NCO["TXA"]
    else:
        TxNCO = lms7002.NCO["TXB"]

    lms7002.fRef = hardware.fRef
    ui.ConnectButton.setText(hardware.sdrName)

    lms7002.MIMO = 'MIMO'

    # Initial configuration
    ui.InitialisedMessage.setText("Tuning Clock")
    app.processEvents()
    lms7002.CGEN.setCLK(300e6)  # set clock to 300MHz
    startFreq, endFreq, nFreq, centreFreq, spanFreq = getFreq()
    hardware.freqDepVar(startFreq)
    ui.InitialisedMessage.setText("Tuning SXT")
    app.processEvents()

    startFreq = float(startFreq * 1e6)
    lms7002.SX['T'].setFREQ(startFreq)  # why is this done only at startFreq?

    # Make ADC and DAC clocks equal
    ui.InitialisedMessage.setText("Setting up RSSI")
    app.processEvents()
    lms7002.CGEN.EN_ADCCLKH_CLKGN = 0  # set ADC clock to F_CLKH and DAC clock to F_CLKL
    lms7002.CGEN.CLKH_OV_CLKL_CGEN = 2

    cal = lms7002.calibration

    # configure Rx A
    RxTSPA.GCORRQ = 2047  # set gain corrector to maximum
    RxTSPA.GCORRI = 2047
    RxTSPA.AGC_MODE = 'RSSI'
    RxTSPA.AGC_BYP = 'USE'
    RxTSPA.RSSI_MODE = 'RSSI'
    # configure Rx B
    RxTSPB.GCORRQ = 2047
    RxTSPB.GCORRI = 2047
    RxTSPB.AGC_MODE = 'RSSI'
    RxTSPB.AGC_BYP = 'USE'
    RxTSPB.RSSI_MODE = 'RSSI'

    TBB.CG_IAMP_TBB = hardware.iAmp

    # Tx calibration
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

    TRF.EN_LOOPB_TXPAD_TRF = 0
    TRF.L_LOOPB_TXPAD_TRF = 0
    TRF.PD_TLOBUF_TRF = 0
    TRF.LOSS_MAIN_TXPAD_TRF = 0
    TRF.SEL_BAND1_TRF = hardware.band1
    TRF.SEL_BAND2_TRF = hardware.band2

    lms7002.SX['R'].EN_G = 0
    lms7002.SX['R'].EN_DIR = 0
    lms7002.SX['T'].PD_LOCH_T2RBUF = 0  # Both RX and TX use the TX PLL

    ui.InitialisedMessage.setText("Loading VNA.hex to MCU")
    app.processEvents()
    mcuProgram()  # Load vna.hex to MCU SRAM for measuring RSSI
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
    ui.graphWidget.setYRange(-60, 60)
    ui.calShortProgress.setValue(0)
    startFreq, endFreq, nFreq, centreFreq, spanFreq = getFreq()
    short.Analyse('', 'A', 'ReturnLoss')
    ui.MeasureRLButton.setEnabled(True)


def calThroughLoss():
    ui.graphWidget.setYRange(-60, 60)
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
    hardware = sdrType()
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
