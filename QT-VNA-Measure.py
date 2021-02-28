import sys
import numpy
from PyQt5 import QtWidgets
import pyqtgraph
from pyLMS7002Soapy import pyLMS7002Soapy as pyLMSS
import Qt_designer_VNA_Gui

limeSDR = pyLMSS.pyLMS7002Soapy(0)
lms7002 = limeSDR.LMS7002

calThreshold = 500  # RSSI threshold to trigger RX DC cal. 250 is ~50% slower than 500

RxChan = 'A'
TxChan = 'A'

class Measurement():
    '''Create measurements of Amplitude and Phase for lists of n frequencies (MHz)'''

    def __init__(self, Calibration, startFreq = 1250, endFreq = 1350, nFreq = 10):
        # Set shortcuts to the pySoapy modules
        self.TxTSP = lms7002.TxTSP[TxChan]
        self.TRF = lms7002.TRF[TxChan]
        self.cal = lms7002.calibration

    def Analyse(self, Calibration, Rx, measType):
        # Measure received signal power and phase.  Phase is not used anywhere at present.
        # Short circuit for Reflection measurements and back-back connection for Through.
        self.measType = measType
        self.res = []
        self.resPhase = []
        self.pgaGains = [] # stores pga gains set per Freq during calibration
        self.lnaGains = [] # stores lna gains set per Freq during calibration
        self.refPhase = 0
        calRSSIbefore = []
        calRSSIafter = []
        power = []
        freq = []
        
        startFreq, endFreq, nFreq, centreFreq, spanFreq = getFreq()
        nFreq += 1 # to give equal frequency spacing around centre freq
        self.freqs = numpy.linspace(startFreq, endFreq, nFreq)
                    
        if limeSDR.boardName != "LimeSDRMini":
            if endFreq > 1500: # above 1.5GHz switch Lime-USB to high band input
                LNA = 'LNAH'
            else:
                LNA = 'LNAL'

        # valid MAC values are [1,2,'A','B','R','RX','T','TX']. Tells MCU which Rx to measure.
        lms7002.MAC = Rx

        for i in range(0, len(self.freqs)):
            f = self.freqs[i] * 1e6

            if Calibration !='': # Set gains for DUT measurement to the calibrated values
                pgaGain = Calibration.pgaGains[i]
                lnaGain = Calibration.lnaGains[i]
                lms7002.RBB[Rx].G_PGA_RBB = pgaGain
                lms7002.RFE[Rx].G_LNA_RFE = lnaGain

            # set freq and cal rx DC offset. Both Tx and Rx freqs since it's using the Tx Osc?
            lms7002.verbose = 0
            lms7002.SX['T'].setFREQ(f)
            lms7002.SX['T'].PD_LOCH_T2RBUF = 0

            # set transmit and receive for testing? loopback?
            syncPhase(lms7002, Rx)

            if Calibration == "":
                # set Rx gain, return the set values and append them to lists
                pgaGain, lnaGain = adjustRxGain(lms7002, Rx)
                self.pgaGains.append(pgaGain)
                self.lnaGains.append(lnaGain)

            # Check residual RSSI (DC offset?) at the set rx gain
            self.TRF.PD_TXPAD_TRF = 1 # Turn off TXPAD
            calRSSI = lms7002.RxTSP[Rx].RSSI
            calRSSIbefore.append(calRSSI)
            if calRSSI > calThreshold:
                self.cal.rxDCLO(Rx, LNA, lnaGain=lnaGain, pgaGain=pgaGain) # takes about 0.9s
                calRSSI = lms7002.RxTSP[Rx].RSSI
            calRSSIafter.append(calRSSI)
            self.TRF.PD_TXPAD_TRF = 0 # Turn on TXPAD

            # get the (average of 3) RSSI value from the LimeSDR
            self.TxTSP.CMIX_BYP = 'USE'
            lms7002.RxTSP[Rx].GC_BYP = 'USE' # turn on gain corrector block
            lms7002.RxTSP[Rx].GCORRQ = 0 # set Q channel gain to zero.
            rssi = 1.0 * mcuRSSI()
            self.TxTSP.CMIX_BYP = 'BYP'
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
                    phase = mcuPhase(Rx) - self.refPhase # set cal phase ref=zero at start freq
                else:
                    phase = mcuPhase(Rx) - Calibration.refPhase
                self.resPhase.append(phase)
            else:
                self.resPhase.append(0)

            lms7002.verbose = 1000

            # plot the results on the GUI graphwidget and update the progress indicators.
            freq.append(self.freqs[i])
            progress = int((i+1)*100/len(self.freqs))
            if Calibration == '':
                CalPower = 20 * numpy.log10(calRSSI) # set to residual value so that log10 is finite
            else:
                CalPower = 20 * numpy.log10(Calibration.res[i])    
            power.append(DutPower-CalPower) 
            if measType == 'ReturnLoss':
                ui.calShortProgress.setValue(progress)
                rlCurve.setData(freq, power)
            else:
                ui.calThroughProgress.setValue(progress)
                throCurve.setData(freq, power)
            pyqtgraph.QtGui.QApplication.processEvents() # force GUI update

        if ui.SaveBox.isChecked():
            reference = str(self) # a unique filename reference for the measurement instance
            writeDataFile(reference[32:-1], self.measType, self.freqs, self.res, self.resPhase, calRSSIbefore, calRSSIafter)

class Marker():
    '''Create amplitude and frequency markers using infinite lines'''

    def __init__(self, markerType, markerValue, labelPosition):
        self.Type = markerType
        self.Value = markerValue
        self.lPos = labelPosition
        self.line = ui.graphWidget.addLine()

    def createLine(self):
        if Marker1.Value == 0: # first run, so set freq markers to GUI values
            Marker1.Value = ui.Marker1.value()
            Marker2.Value = ui.Marker2.value()
            Marker3.Value = ui.Marker3.value()
        markerPen=pyqtgraph.mkPen(color='g', width=0.5)
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
    # Phase is measured by setting I channel gain to zero using RxTSP gain corrector
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
    # lms7002 calls pyLMSS.pyLMS7002Soapy
    TRF = lms7002.TRF[TxChan]
    RFE = lms7002.RFE[Rx]
    SXT = lms7002.SX['T']
    SXT.PD_FDIV = 1
    TRF.PD_TLOBUF_TRF = 1
    RFE.PD_QGEN_RFE = 1
    TRF.PD_TLOBUF_TRF = 0
    RFE.PD_QGEN_RFE = 0
    SXT.PD_FDIV = 0

def adjustRxGain(lms7002, Rx):
    # returns the pga and lna gain values for optimum dynamic range.
    RBB = lms7002.RBB[Rx]
    RFE = lms7002.RFE[Rx]
    RxTSP = lms7002.RxTSP[Rx]
    TxTSP = lms7002.TxTSP[TxChan]

    TxTSP.CMIX_BYP = 'USE'
    RxTSP.GC_BYP = 'USE'
    RxTSP.GCORRQ = 0
    pgaGain = 0
    pgaStep = 16
    if limeSDR.boardName == "LimeSDRMini":
        lnaGain = 6
    else:
        lnaGain = 8

    while pgaStep > 0:
        RBB.G_PGA_RBB = pgaGain + pgaStep
        RFE.G_LNA_RFE = lnaGain
        if mcuRSSI() < 50e3:
            pgaGain += pgaStep
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
        rssia = after [i]
        txtRes += str(f) + '\t' + str(y) + '\t' + str(phase) + '\t' + str(rssib) + '\t' + str(rssia) + '\n'
    outFile.write(txtRes)
    outFile.close()

# Connect to SDR and initialise
def ConnectSDR():
    # Set up the shortcuts to pySoapy modules
    TBB = lms7002.TBB[TxChan]
    TRF = lms7002.TRF[TxChan]
    RxTSPA = lms7002.RxTSP['A']
    RxTSPB = lms7002.RxTSP['B']
    TxTSP = lms7002.TxTSP[TxChan]
    if TxChan == 'A':
        TxNCO = lms7002.NCO["TXA"]
    else:
        TxNCO = lms7002.NCO["TXB"]

    if limeSDR.boardName == "LimeSDRMini":
        isMini = "True"
        lms7002.fRef = 40e6
    else:
        isMini = "False"
        lms7002.fRef = 30.72e6

    ui.ConnectButton.setText(limeSDR.boardName)

    lms7002.MIMO = 'MIMO'

    # Initial configuration
    ui.InitialisedMessage.setText("Tuning Clock")
    app.processEvents()
    lms7002.CGEN.setCLK(300e6) # set clock to 300MHz
    startFreq, endFreq, nFreq, centreFreq, spanFreq = getFreq()
    ui.InitialisedMessage.setText("Tuning SXT")
    app.processEvents()

    startFreq = float(startFreq * 1e6)
    lms7002.SX['T'].setFREQ(startFreq) # why is this done only at startFreq?

    # Make ADC and DAC clocks equal
    ui.InitialisedMessage.setText("Setting up RSSI")
    app.processEvents()
    lms7002.CGEN.EN_ADCCLKH_CLKGN = 0
    lms7002.CGEN.CLKH_OV_CLKL_CGEN = 2

    cal = lms7002.calibration

    # configure Rx A
    RxTSPA.GCORRQ = 2047
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

    if isMini:
        TBB.CG_IAMP_TBB = 5
    else:
        TBB.CG_IAMP_TBB = 15

    # configure Tx
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

    NCOfreq = 50e3 # Numerically Controlled Oscillator to 50kHz.
    TxNCO.MODE = 0
    TxNCO.setNCOFrequency(0, NCOfreq)
    TxNCO.SEL = 0

    TRF.EN_LOOPB_TXPAD_TRF = 0
    TRF.L_LOOPB_TXPAD_TRF = 0
    TRF.PD_TLOBUF_TRF = 0

    if isMini:
        TRF.LOSS_MAIN_TXPAD_TRF = 0
        TRF.SEL_BAND1_TRF = 1
        TRF.SEL_BAND2_TRF = 0
        limeSDR.configureAntenna(startFreq)
    else:
        TRF.LOSS_MAIN_TXPAD_TRF = 0
        TRF.SEL_BAND1_TRF = 0 # < 1.5GHz Transmit channel off ############################## needs thought
        TRF.SEL_BAND2_TRF = 1 # >= 1.5GHz Transmit channel on

    lms7002.SX['R'].EN_G = 0
    lms7002.SX['R'].EN_DIR = 0
    lms7002.SX['T'].PD_LOCH_T2RBUF = 0  # Both RX and TX use the TX PLL

    ui.InitialisedMessage.setText("Loading VNA.hex to MCU")
    app.processEvents()
    mcuProgram()  # Load vna.hex to MCU SRAM for measuring RSSI

    ui.InitialisedMessage.setText("Calibrating RX DC")
    app.processEvents()

    LNA = 'LNAL'
    if isMini:
        lnaGain = 6
    else:
        lnaGain = 8
        if endFreq > 1500: # above 1.5GHz switch Lime-USB to high band input
            LNA = 'LNAH'
    pgaGain = 16

    TRF.PD_TXPAD_TRF = 1  # Turn off TXPAD while calibrating RX DC
    cal.rxDCLO(RxChan, LNA, lnaGain=lnaGain, pgaGain=pgaGain)  # Calibrate RX DC
    TRF.PD_TXPAD_TRF = 0  # Turn on TXPAD

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
    ui.calShortProgress.setValue(0)
    startFreq, endFreq, nFreq, centreFreq, spanFreq = getFreq()
    short.Analyse('', 'A', 'ReturnLoss')
    ui.MeasureRLButton.setEnabled(True)

def calThroughLoss():
    ui.calThroughProgress.setValue(0)
    startFreq, endFreq, nFreq, centreFreq, spanFreq = getFreq()
    through.Analyse('', 'B', 'ThroughLoss')
    ui.MeasureThroughButton.setEnabled(True)

def measReturnLoss():
    while ui.Repeat.isChecked():
        DutRefl.Analyse(short, 'A', 'ReturnLoss')
        pyqtgraph.QtGui.QApplication.processEvents()
        if not ui.Repeat.isChecked(): break
    else:
        DutRefl.Analyse(short, 'A', 'ReturnLoss')

def measThroughLoss():
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
    ui.graphWidget.setYRange(-35, 5)
    ui.graphWidget.setXRange(1250, 1350)
    ui.graphWidget.addLegend(offset=2)
    rlCurve = ui.graphWidget.plot([], [], name='Return Loss', pen='y', width=5)
    throCurve = ui.graphWidget.plot([], [], name='Through Loss', pen='c', width=3)

    # instantiate measurements and markers
    short = Measurement(0,1250,1350,10)
    through = Measurement(0,1250,1350,10)
    DutRefl = Measurement(0,1250,1350,10)
    DutThro = Measurement(0,1250,1350,10)

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